use crate::error::{Result, ResultHelper};
use crate::game_state::DT;
use crate::{components as comps, game_state, ui, utils};
use allegro::*;
use allegro_font::*;
use na::{
	Isometry3, Matrix4, Perspective3, Point2, Point3, RealField, Rotation2, Rotation3, Similarity3,
	Unit, UnitQuaternion, Vector2, Vector3, Vector4,
};
use nalgebra as na;
use rand::prelude::*;
use rapier3d::dynamics::{
	CCDSolver, FixedJointBuilder, ImpulseJointSet, IntegrationParameters, IslandManager,
	MassProperties, MultibodyJointSet, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
	SpringJointBuilder,
};
use rapier3d::geometry::{
	Ball, ColliderBuilder, ColliderSet, CollisionEvent, ContactPair, DefaultBroadPhase,
	NarrowPhase, SharedShape,
};
use rapier3d::pipeline::{ActiveEvents, EventHandler, PhysicsPipeline, QueryPipeline};
use slhack::{controls, scene, sprite, ui as slhack_ui};

use std::collections::HashMap;
use std::f32::consts::PI;
use std::sync::RwLock;

fn get_dirs(rot: UnitQuaternion<f32>) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>)
{
	let forward = -(rot * Vector3::z());
	let right = rot * Vector3::x();
	let up = rot * Vector3::y();

	(forward, right, up)
}

pub struct PhysicsEventHandler
{
	collision_events: RwLock<Vec<(CollisionEvent, Option<ContactPair>)>>,
	contact_force_events: RwLock<Vec<(f32, ContactPair)>>,
}

impl PhysicsEventHandler
{
	pub fn new() -> Self
	{
		Self {
			collision_events: RwLock::new(vec![]),
			contact_force_events: RwLock::new(vec![]),
		}
	}
}

impl EventHandler for PhysicsEventHandler
{
	fn handle_collision_event(
		&self, _bodies: &RigidBodySet, _colliders: &ColliderSet, event: CollisionEvent,
		contact_pair: Option<&ContactPair>,
	)
	{
		let mut events = self.collision_events.write().unwrap();
		events.push((event, contact_pair.cloned()));
	}

	fn handle_contact_force_event(
		&self, _dt: f32, _bodies: &RigidBodySet, _colliders: &ColliderSet,
		contact_pair: &ContactPair, total_force_magnitude: f32,
	)
	{
		self.contact_force_events
			.write()
			.unwrap()
			.push((total_force_magnitude, contact_pair.clone()));
	}
}

pub struct Physics
{
	rigid_body_set: RigidBodySet,
	collider_set: ColliderSet,
	integration_parameters: IntegrationParameters,
	physics_pipeline: PhysicsPipeline,
	island_manager: IslandManager,
	broad_phase: DefaultBroadPhase,
	narrow_phase: NarrowPhase,
	impulse_joint_set: ImpulseJointSet,
	multibody_joint_set: MultibodyJointSet,
	ccd_solver: CCDSolver,
}

impl Physics
{
	fn new() -> Self
	{
		Self {
			rigid_body_set: RigidBodySet::new(),
			collider_set: ColliderSet::new(),
			integration_parameters: IntegrationParameters {
				dt: game_state::DT,
				num_solver_iterations: 10,
				//num_internal_pgs_iterations: 10,
				//contact_damping_ratio: 0.01,
				//contact_natural_frequency: 60.,
				//normalized_max_corrective_velocity: 100.,
				//joint_damping_ratio: 0.01,
				//joint_natural_frequency: 1e8,
				..IntegrationParameters::default()
			},
			physics_pipeline: PhysicsPipeline::new(),
			island_manager: IslandManager::new(),
			broad_phase: DefaultBroadPhase::new(),
			narrow_phase: NarrowPhase::new(),
			impulse_joint_set: ImpulseJointSet::new(),
			multibody_joint_set: MultibodyJointSet::new(),
			ccd_solver: CCDSolver::new(),
		}
	}

	fn step(&mut self, event_handler: &PhysicsEventHandler)
	{
		let gravity = Vector3::zeros();
		self.physics_pipeline.step(
			&gravity,
			&self.integration_parameters,
			&mut self.island_manager,
			&mut self.broad_phase,
			&mut self.narrow_phase,
			&mut self.rigid_body_set,
			&mut self.collider_set,
			&mut self.impulse_joint_set,
			&mut self.multibody_joint_set,
			&mut self.ccd_solver,
			&(),
			event_handler,
		);
	}
}

pub struct Game
{
	map: Map,
	subscreens: ui::SubScreens,
}

impl Game
{
	pub fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		Ok(Self {
			map: Map::new(state)?,
			subscreens: ui::SubScreens::new(state),
		})
	}

	pub fn logic(
		&mut self, state: &mut game_state::GameState,
	) -> Result<Option<game_state::NextScreen>>
	{
		if self.subscreens.is_empty()
		{
			state.hs.hide_mouse = true;
			self.map.logic(state)
		}
		else
		{
			Ok(None)
		}
	}

	pub fn input(
		&mut self, event: &Event, state: &mut game_state::GameState,
	) -> Result<Option<game_state::NextScreen>>
	{
		match *event
		{
			Event::MouseAxes { x, y, .. } =>
			{
				if state.hs.track_mouse
				{
					let (x, y) = state.hs.transform_mouse(x as f32, y as f32);
					state.hs.mouse_pos = Point2::new(x as i32, y as i32);
				}
			}
			_ => (),
		}
		if self.subscreens.is_empty()
		{
			let mut in_game_menu = false;
			let handled = false; // In case there's other in-game UI to handle this.
			if state
				.hs
				.game_ui_controls
				.get_action_state(slhack_ui::UIAction::Cancel)
				> 0.5
			{
				in_game_menu = true;
			}
			else if !handled
			{
				let res = self.map.input(event, state);
				if let Ok(Some(game_state::NextScreen::InGameMenu)) = res
				{
					in_game_menu = true;
				}
				else
				{
					return res;
				}
			}
			if in_game_menu
			{
				self.subscreens
					.push(ui::SubScreen::InGameMenu(ui::InGameMenu::new(state)));
				self.subscreens.reset_transition(state);
			}
		}
		else
		{
			if let Some(action) = self.subscreens.input(state, event)?
			{
				match action
				{
					ui::Action::MainMenu =>
					{
						return Ok(Some(game_state::NextScreen::Menu));
					}
					_ => (),
				}
			}
			if self.subscreens.is_empty()
			{
				state.controls.clear_action_states();
			}
		}
		Ok(None)
	}

	pub fn draw(&mut self, state: &mut game_state::GameState) -> Result<()>
	{
		if !self.subscreens.is_empty()
		{
			state
				.hs
				.core
				.clear_to_color(Color::from_rgb_f(0.0, 0.0, 0.0));
			self.subscreens.draw(state);
		}
		else
		{
			self.map.draw(state)?;
		}
		Ok(())
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		self.subscreens.resize(state);
	}
}

pub fn spawn_robot(
	pos: Point3<f32>, physics: &mut Physics, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene = "data/robot1.glb";
	game_state::cache_scene(state, scene)?;
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Scene {
			scene: scene.to_string(),
		},
		comps::Controller::new(),
		comps::AI::new(),
	));
	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.angular_damping(10.)
		.linear_damping(5.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.5)
		.restitution(0.1)
		.mass(4.0)
		.friction(0.)
		.user_data(entity.to_bits().get() as u128)
		//.active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
		.build();
	let ball_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		ball_body_handle,
		&mut physics.rigid_body_set,
	);
	world.insert_one(
		entity,
		comps::Physics {
			handle: ball_body_handle,
		},
	)?;
	Ok(entity)
}

pub fn spawn_player(
	pos: Point3<f32>, physics: &mut Physics, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene = "data/test.glb";
	game_state::cache_scene(state, scene)?;
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Controller::new(),
		comps::Scene {
			scene: scene.to_string(),
		},
		comps::Grippers::new(),
		comps::Light {
			color: Color::from_rgb_f(1., 1., 1.),
			intensity: 100.,
			static_: false,
		},
	));
	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.angular_damping(10.)
		.linear_damping(5.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.25)
		.restitution(0.1)
		//.mass(4.0)
		.mass_properties(MassProperties::new(
			Point3::origin(),
			4.0,
			1000.0f32 * Vector3::new(1., 1., 1.),
		))
		.friction(0.)
		.user_data(entity.to_bits().get() as u128)
		//.active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
		.build();
	let ball_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		ball_body_handle,
		&mut physics.rigid_body_set,
	);
	world.insert_one(
		entity,
		comps::Physics {
			handle: ball_body_handle,
		},
	)?;
	Ok(entity)
}

pub fn spawn_gripper(
	pos: Point3<f32>, vel: Vector3<f32>, parent: hecs::Entity, physics: &mut Physics,
	world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene = "data/gripper.glb";
	game_state::cache_scene(state, scene)?;
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Scene {
			scene: scene.to_string(),
		},
		comps::Gripper::new(parent),
		comps::Light {
			color: Color::from_rgb_f(0., 0., 1.),
			intensity: 100.,
			static_: false,
		},
	));
	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.angular_damping(1.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.2)
		.restitution(0.8)
		.mass(1.0)
		.user_data(entity.to_bits().get() as u128)
		.active_events(ActiveEvents::COLLISION_EVENTS)
		.build();
	let ball_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		ball_body_handle,
		&mut physics.rigid_body_set,
	);

	physics
		.rigid_body_set
		.get_mut(ball_body_handle)
		.unwrap()
		.apply_impulse(vel, true);

	world.insert_one(
		entity,
		comps::Physics {
			handle: ball_body_handle,
		},
	)?;
	Ok(entity)
}

pub fn spawn_connector(
	start: hecs::Entity, end: hecs::Entity, start_offset: Vector3<f32>, end_offset: Vector3<f32>,
	world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	game_state::cache_scene(state, "data/connector.glb")?;
	let entity = world.spawn((comps::Connector::new(start, end, start_offset, end_offset),));
	Ok(entity)
}

pub fn spawn_obj(pos: Point3<f32>, world: &mut hecs::World) -> Result<hecs::Entity>
{
	let entity = world.spawn((comps::Position::new(pos, UnitQuaternion::identity()),));
	Ok(entity)
}

pub fn spawn_light(
	pos: Point3<f32>, light: comps::Light, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let entity = world.spawn((comps::Position::new(pos, UnitQuaternion::identity()), light));
	Ok(entity)
}

pub fn spawn_level(
	scene_name: &str, state: &mut game_state::GameState, physics: &mut Physics,
	world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	game_state::cache_scene(state, scene_name)?;

	let entity = world.spawn((
		comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
		comps::Scene {
			scene: scene_name.to_string(),
		},
	));

	let level_scene = state.get_scene(scene_name).unwrap();
	let mut vertices = vec![];
	let mut indices = vec![];
	for object in &level_scene.objects
	{
		match &object.kind
		{
			scene::ObjectKind::MultiMesh { meshes } =>
			{
				let mut index_offset = 0;
				for mesh in meshes
				{
					for vtx in &mesh.vtxs
					{
						vertices.push(Point3::new(vtx.x, vtx.y, vtx.z));
					}
					for idxs in mesh.idxs.chunks(3)
					{
						indices.push([
							idxs[0] as u32 + index_offset,
							idxs[1] as u32 + index_offset,
							idxs[2] as u32 + index_offset,
						]);
					}
					index_offset += mesh.vtxs.len() as u32;
				}
			}
			_ => (),
		}
	}
	let rigid_body = RigidBodyBuilder::fixed()
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::trimesh(vertices, indices)?
		.user_data(entity.to_bits().get() as u128)
		.build();
	let rigid_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		rigid_body_handle,
		&mut physics.rigid_body_set,
	);
	world.insert_one(
		entity,
		comps::Physics {
			handle: rigid_body_handle,
		},
	)?;
	Ok(entity)
}

struct Map
{
	world: hecs::World,
	physics: Physics,
	camera_target: comps::Position,
	player: hecs::Entity,
	level: hecs::Entity,
}

impl Map
{
	fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		let mut world = hecs::World::new();
		spawn_obj(Point3::new(0., 0., 0.), &mut world)?;

		// ???
		game_state::cache_scene(state, "data/test_level.glb")?;
		state.cache_bitmap("data/level_lightmap.png")?;

		game_state::cache_scene(state, "data/sphere.glb")?;
		game_state::cache_scene(state, "data/test.obj")?;

		let level_scene = state.get_scene("data/test_level.glb").unwrap();
		for object in &level_scene.objects
		{
			if let scene::ObjectKind::Light { color, intensity } = object.kind
			{
				spawn_light(
					object.position,
					comps::Light {
						color: color,
						intensity: intensity / 50.,
						static_: true,
					},
					&mut world,
				)?;
			}
		}

		let mut physics = Physics::new();
		spawn_robot(Point3::new(2.5, 2.5, 1.), &mut physics, &mut world, state)?;
		let player = spawn_player(Point3::new(2., 2.5, 4.), &mut physics, &mut world, state)?;
		let level = spawn_level("data/test_level.glb", state, &mut physics, &mut world)?;

		Ok(Self {
			world: world,
			physics: physics,
			camera_target: comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
			player: player,
			level: level,
		})
	}

	fn logic(&mut self, state: &mut game_state::GameState)
	-> Result<Option<game_state::NextScreen>>
	{
		let mut to_die = vec![];

		// Position snapshotting.
		for (_, position) in self.world.query::<&mut comps::Position>().iter()
		{
			position.snapshot();
		}
		self.camera_target.snapshot();

		// Player.
		if self.world.contains(self.player)
		{
			let position = self.world.get::<&comps::Position>(self.player).unwrap();
			let mut controller = self
				.world
				.get::<&mut comps::Controller>(self.player)
				.unwrap();

			let right_left = state
				.controls
				.get_action_state(game_state::Action::MoveRight)
				- state
					.controls
					.get_action_state(game_state::Action::MoveLeft);
			let up_down = state.controls.get_action_state(game_state::Action::MoveUp)
				- state
					.controls
					.get_action_state(game_state::Action::MoveDown);

			let rot_right_left = state
				.controls
				.get_action_state(game_state::Action::RotateRight)
				- state
					.controls
					.get_action_state(game_state::Action::RotateLeft);
			let rot_up_down = state
				.controls
				.get_action_state(game_state::Action::RotateUp)
				- state
					.controls
					.get_action_state(game_state::Action::RotateDown);

			controller.want_move = Vector3::new(right_left, up_down, 0.);
			controller.want_rotate = Vector3::new(-rot_up_down, -rot_right_left, 0.);

			for (idx, action) in [game_state::Action::GripLeft, game_state::Action::GripRight]
				.iter()
				.enumerate()
			{
				controller.want_gripper[idx] = state.controls.get_action_state(*action) > 0.5;
				state.controls.clear_action_state(*action);
			}

			self.camera_target.pos = position.pos;
			self.camera_target.rot = position.rot;
			self.camera_target.scale = position.scale;
		}

		// AI.
		for (_, (position, controller, ai)) in self
			.world
			.query::<(&comps::Position, &mut comps::Controller, &mut comps::AI)>()
			.iter()
		{
			let mut new_state = None;
			match ai.state
			{
				comps::AIState::Idle =>
				{
					if let Ok(player_position) = self.world.get::<&comps::Position>(self.player)
					{
						if (player_position.pos - position.pos).norm() < 4.
						{
							new_state = Some(comps::AIState::Attacking(self.player))
						}
					}
				}
				comps::AIState::Attacking(target) =>
				{
					if let Ok(target_position) = self.world.get::<&comps::Position>(target)
					{
						let (forward, right, up) = get_dirs(position.rot);
						let diff = target_position.pos - position.pos;
						let dir = diff.normalize();
						let rot_speed = 5.;
						controller.want_rotate.x = rot_speed * dir.dot(&up);
						controller.want_rotate.y = -rot_speed * dir.dot(&right);

						controller.want_move.z = 0.;
						if dir.dot(&forward) > 0.99
						{
							if diff.norm() > 3.
							{
								controller.want_move.z = 1.;
							}
							else if diff.norm() < 1.
							{
								controller.want_move.z = -1.;
							}
						}
					}
				}
			}
			if let Some(new_state) = new_state
			{
				ai.state = new_state;
			}
		}

		// Friction + force resetting.
		for (_, (position, physics)) in self
			.world
			.query::<(&comps::Position, &mut comps::Physics)>()
			.iter()
		{
			let f = 0.9;
			let body = &mut self.physics.rigid_body_set[physics.handle];
			body.reset_forces(true);
			body.reset_torques(true);
			//body.add_force(-f * body.velocity_at_point(&position.pos), true);
			//let f = 0.5;
			//dbg!(body.angvel());
			//body.add_torque(-f * body.angvel(), true);
		}

		// Controller.
		for (_, (controller, physics)) in self
			.world
			.query::<(&mut comps::Controller, &comps::Physics)>()
			.iter()
		{
			let body = self.physics.rigid_body_set.get_mut(physics.handle).unwrap();

			let rot = body.rotation().clone();
			let (forward, right, up) = get_dirs(rot);

			body.add_force(
				16. * (controller.want_move.x * right
					+ controller.want_move.y * up
					+ controller.want_move.z * forward),
				true,
			);
			//body.apply_torque_impulse(DT * (rot * controller.want_rotate), true);
			body.set_angvel(
				DT * 3. * (rot * controller.want_rotate) + body.angvel(),
				true,
			);
		}

		// Grippers.
		let mut to_spawn = vec![];
		for (id, (controller, position, grippers)) in self
			.world
			.query::<(
				&mut comps::Controller,
				&comps::Position,
				&mut comps::Grippers,
			)>()
			.iter()
		{
			for (idx, (want_gripper, gripper, time_to_grip, offset)) in itertools::izip!(
				&controller.want_gripper,
				&mut grippers.grippers,
				&mut grippers.time_to_grip,
				&grippers.offsets
			)
			.enumerate()
			{
				if *want_gripper
				{
					if let Some(gripper_id) = gripper
					{
						if self.world.contains(*gripper_id)
						{
							to_die.push(*gripper_id);
						}
						*gripper = None;
					}
					else
					{
						if state.hs.time() >= *time_to_grip
						{
							to_spawn.push((id, position.pos, position.rot, *offset, idx));
							*time_to_grip = state.hs.time() + 0.5;
						}
					}
				}
			}
		}
		for (id, pos, rot, offset, idx) in to_spawn
		{
			let (forward, _, _) = get_dirs(rot);
			let gripper = spawn_gripper(
				pos + rot * offset,
				10. * forward,
				id,
				&mut self.physics,
				&mut self.world,
				state,
			)?;
			spawn_connector(
				id,
				gripper,
				offset,
				Vector3::zeros(),
				&mut self.world,
				state,
			)?;
			let mut grippers = self.world.get::<&mut comps::Grippers>(id).unwrap();
			grippers.grippers[idx] = Some(gripper);
		}

		// Physics.
		let handler = PhysicsEventHandler::new();
		self.physics.step(&handler);

		for (event, _contact_pair) in handler.collision_events.try_read().unwrap().iter()
		{
			//if let Some(contact_pair) = contact_pair
			//	&& contact_pair.has_any_active_contact
			if let CollisionEvent::Started(collider_handle_1, collider_handle_2, _) = event
			{
				//let collider_handle_1 = contact_pair.collider1;
				//let collider_handle_2 = contact_pair.collider2;
				let collider1 = self.physics.collider_set.get(*collider_handle_1).unwrap();
				let collider2 = self.physics.collider_set.get(*collider_handle_2).unwrap();
				let body_handle_1 = collider1.parent().unwrap();
				let body_handle_2 = collider2.parent().unwrap();
				let id1 = hecs::Entity::from_bits(collider1.user_data as u64).unwrap();
				let id2 = hecs::Entity::from_bits(collider2.user_data as u64).unwrap();

				let (_, id, level_body_handle, body_handle) = if id1 == self.level
				{
					(id1, id2, body_handle_1, body_handle_2)
				}
				else if id2 == self.level
				{
					(id1, id2, body_handle_2, body_handle_1)
				}
				else
				{
					if self.world.get::<&comps::Gripper>(id1).is_ok()
					{
						to_die.push(id1);
					}
					if self.world.get::<&comps::Gripper>(id2).is_ok()
					{
						to_die.push(id2);
					}
					continue;
				};

				if let Some(gripper) = self.world.query_one::<&comps::Gripper>(id).unwrap().get()
				{
					let body = self.physics.rigid_body_set.get_mut(body_handle).unwrap();

					let joint = FixedJointBuilder::new()
						.local_anchor1(Point3::new(0., 0., 0.))
						.local_anchor2(Point3::origin() + body.translation());
					self.physics.impulse_joint_set.insert(
						body_handle,
						level_body_handle,
						joint,
						true,
					);

					if let Some(physics) = self
						.world
						.query_one::<&mut comps::Physics>(gripper.parent)
						.unwrap()
						.get()
					{
						let joint = SpringJointBuilder::new(0.1, 30., 10.)
							.local_anchor1(Point3::origin())
							.local_anchor2(Point3::origin());
						self.physics.impulse_joint_set.insert(
							body_handle,
							physics.handle,
							joint,
							true,
						);
					}
				}
			}
		}

		for (_id, (position, physics)) in self
			.world
			.query::<(&mut comps::Position, &comps::Physics)>()
			.iter()
		{
			let body = &self.physics.rigid_body_set[physics.handle];
			position.pos = Point3::from(*body.translation());
			position.rot = *body.rotation();
		}

		// Connector upkeep
		for (id, connector) in self.world.query::<&comps::Connector>().iter()
		{
			if !self.world.contains(connector.start) || !self.world.contains(connector.end)
			{
				to_die.push(id);
			}
		}

		// Remove dead entities
		to_die.sort();
		to_die.dedup();
		for id in to_die
		{
			if let Ok(physics) = self.world.get::<&comps::Physics>(id)
			{
				self.physics.rigid_body_set.remove(
					physics.handle,
					&mut self.physics.island_manager,
					&mut self.physics.collider_set,
					&mut self.physics.impulse_joint_set,
					&mut self.physics.multibody_joint_set,
					true,
				);
			}
			self.world.despawn(id)?;
		}

		Ok(None)
	}

	fn input(
		&mut self, _event: &Event, _state: &mut game_state::GameState,
	) -> Result<Option<game_state::NextScreen>>
	{
		Ok(None)
	}

	fn make_project(&self, state: &game_state::GameState) -> Perspective3<f32>
	{
		utils::projection_transform(state.hs.buffer_width(), state.hs.buffer_height(), PI / 3.)
	}

	fn camera_pos(&self, alpha: f32) -> Point3<f32>
	{
		self.camera_target.draw_pos(alpha)
	}

	fn make_camera(&self, alpha: f32) -> Isometry3<f32>
	{
		Isometry3 {
			rotation: self.camera_target.draw_rot(alpha),
			translation: self.camera_target.draw_pos(alpha).coords.into(),
		}
		.inverse()
	}

	fn draw(&mut self, state: &mut game_state::GameState) -> Result<()>
	{
		let alpha = state.hs.alpha;
		let project = self.make_project(state);
		let camera = self.make_camera(alpha);

		// Forward pass.
		state
			.hs
			.core
			.use_projection_transform(&utils::mat4_to_transform(project.to_homogeneous()));
		state
			.hs
			.core
			.use_transform(&utils::mat4_to_transform(camera.to_homogeneous()));
		state
			.deferred_renderer
			.as_mut()
			.unwrap()
			.begin_forward_pass(&state.hs.core)?;
		state
			.hs
			.core
			.use_shader(Some(state.forward_shader.as_ref().unwrap()))
			.unwrap();

		let shift = Isometry3::new(Vector3::zeros(), Vector3::zeros()).to_homogeneous();

		state
			.hs
			.core
			.use_transform(&utils::mat4_to_transform(camera.to_homogeneous() * shift));
		state
			.hs
			.core
			.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift))
			.ok();

		let material_mapper = |material: &scene::Material<game_state::MaterialKind>,
		                       texture_name: &str|
		 -> slhack::error::Result<&Bitmap> {
			if material.desc.two_sided
			{
				unsafe {
					gl::Disable(gl::CULL_FACE);
				}
			}
			else
			{
				unsafe {
					gl::Enable(gl::CULL_FACE);
				}
			}
			if !material.desc.lightmap.is_empty()
			{
				state
					.hs
					.core
					.set_shader_sampler(
						"lightmap",
						state.get_bitmap(&material.desc.lightmap).into_slhack()?,
						1,
					)
					.ok();
			}
			state.get_bitmap(texture_name).into_slhack()
		};

		for (_, (position, scene)) in self
			.world
			.query::<(&comps::Position, &comps::Scene)>()
			.iter()
		{
			let shift = Isometry3 {
				translation: position.draw_pos(alpha).coords.into(),
				rotation: position.draw_rot(alpha),
			}
			.to_homogeneous();
			let scale = Similarity3::from_scaling(position.draw_scale(alpha)).to_homogeneous();

			state.hs.core.use_transform(&utils::mat4_to_transform(
				camera.to_homogeneous() * shift * scale,
			));
			state
				.hs
				.core
				.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift * scale))
				.ok();

			state.get_scene(&scene.scene).unwrap().draw(
				&state.hs.core,
				&state.hs.prim,
				material_mapper,
			);
		}

		// Connectors
		for (_, connector) in self.world.query::<&comps::Connector>().iter()
		{
			if let (Ok(start_pos), Ok(end_pos)) = (
				self.world.get::<&comps::Position>(connector.start),
				self.world.get::<&comps::Position>(connector.end),
			)
			{
				let start_pos =
					start_pos.draw_pos(alpha) + start_pos.draw_rot(alpha) * connector.start_offset;
				let end_pos =
					end_pos.draw_pos(alpha) + end_pos.draw_rot(alpha) * connector.end_offset;

				let shift = Isometry3 {
					translation: ((start_pos.coords + end_pos.coords) / 2.0).into(),
					rotation: UnitQuaternion::face_towards(&(end_pos - start_pos), &Vector3::y()),
				}
				.to_homogeneous();
				let scale = Matrix4::new_nonuniform_scaling(&Vector3::new(
					1.,
					1.,
					(start_pos - end_pos).norm() / 2.,
				));

				state.hs.core.use_transform(&utils::mat4_to_transform(
					camera.to_homogeneous() * shift * scale,
				));
				state
					.hs
					.core
					.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift * scale))
					.ok();

				state.get_scene("data/connector.glb").unwrap().draw(
					&state.hs.core,
					&state.hs.prim,
					material_mapper,
				);
			}
		}

		// Light pass.
		state.deferred_renderer.as_mut().unwrap().begin_light_pass(
			&state.hs.core,
			state.light_shader.as_ref().unwrap(),
			&utils::mat4_to_transform(project.to_homogeneous()),
			self.camera_pos(alpha),
		)?;

		for (_, (position, light)) in self
			.world
			.query::<(&comps::Position, &comps::Light)>()
			.iter()
		{
			let shift = Isometry3::new(position.draw_pos(alpha).coords, Vector3::zeros());
			let transform = Similarity3::from_isometry(shift, 0.5 * light.intensity.sqrt());
			let light_pos = transform.transform_point(&Point3::origin());

			let (r, g, b) = light.color.to_rgb_f();

			state
				.hs
				.core
				.set_shader_uniform("light_color", &[[r, g, b, 1.0]][..])
				.ok(); //.unwrap();
			state
				.hs
				.core
				.set_shader_uniform("light_pos", &[[light_pos.x, light_pos.y, light_pos.z]][..])
				.ok(); //.unwrap();
			state
				.hs
				.core
				.set_shader_uniform("light_intensity", &[light.intensity][..])
				.ok(); //.unwrap();
			state
				.hs
				.core
				.set_shader_uniform("is_static", &[light.static_ as i32][..])
				.ok(); //.unwrap();

			state.hs.core.use_transform(&utils::mat4_to_transform(
				camera.to_homogeneous() * transform.to_homogeneous(),
			));

			if let Ok(scene) = state.get_scene("data/sphere.glb")
			{
				scene.draw(&state.hs.core, &state.hs.prim, |_, s| {
					state.get_bitmap(s).into_slhack()
				});
			}
		}

		// Final pass.
		state.deferred_renderer.as_mut().unwrap().final_pass(
			&state.hs.core,
			&state.hs.prim,
			state.final_shader.as_ref().unwrap(),
			state.hs.buffer1.as_ref().unwrap(),
		)?;

		state
			.hs
			.core
			.use_shader(Some(state.basic_shader.as_ref().unwrap()))
			.unwrap();
		unsafe {
			gl::Disable(gl::CULL_FACE);
		}
		state.hs.core.set_depth_test(None);
		state
			.hs
			.core
			.set_blender(BlendOperation::Add, BlendMode::One, BlendMode::InverseAlpha);
		Ok(())
	}
}
