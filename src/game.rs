use crate::error::{Result, ResultHelper};
use crate::game_state::DT;
use crate::{components as comps, game_state, ui, utils};
use allegro::*;
use allegro_font::*;
use itertools::Itertools;
use na::{
	Isometry3, Matrix4, Perspective3, Point2, Point3, RealField, Rotation2, Rotation3, Similarity3,
	Unit, UnitQuaternion, Vector2, Vector3, Vector4,
};
use nalgebra as na;
use rand::prelude::*;
use rapier3d::dynamics::{
	CCDSolver, FixedJointBuilder, ImpulseJointHandle, ImpulseJointSet, IntegrationParameters,
	IslandManager, MassProperties, MultibodyJointSet, RigidBodyBuilder, RigidBodyHandle,
	RigidBodySet, SpringJointBuilder,
};
use rapier3d::geometry::{
	Ball, ColliderBuilder, ColliderSet, CollisionEvent, ContactPair, DefaultBroadPhase, Group,
	InteractionGroups, NarrowPhase, SharedShape,
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

const PLAYER_GROUP: Group = Group::GROUP_1;
const GRIPPER_GROUP: Group = Group::GROUP_2;
const BIG_GROUP: Group = Group::GROUP_3;
const SMALL_GROUP: Group = Group::GROUP_4;
const NO_COLLISION: Group = Group::GROUP_5;

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
	let scene_name = "data/robot1.glb";
	game_state::cache_scene(state, scene_name)?;
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Scene::new(scene_name),
		comps::Controller::new(),
		comps::Health::new(55.),
		comps::AI::new(),
		comps::Weapon::new(Vector3::new(0., 0., -1.)),
	));

	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.angular_damping(10.)
		.linear_damping(5.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.5)
		.restitution(0.1)
		.mass(1.0)
		.friction(0.)
		.user_data(entity.to_bits().get() as u128)
		.collision_groups(InteractionGroups::new(BIG_GROUP, PLAYER_GROUP | BIG_GROUP | SMALL_GROUP | GRIPPER_GROUP))
		//.active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
		.build();
	let ball_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		ball_body_handle,
		&mut physics.rigid_body_set,
	);
	world.insert_one(entity, comps::Physics::new(ball_body_handle))?;
	Ok(entity)
}

pub fn attach_gripper_to_parent(
	gripper_id: hecs::Entity, world: &hecs::World, physics: &mut Physics,
)
{
	let mut gripper_query = world
		.query_one::<(&comps::Physics, &mut comps::Position, &mut comps::Gripper)>(gripper_id)
		.unwrap();
	let (gripper_physics, gripper_position, gripper) = gripper_query.get().unwrap();

	if let Some(joint_handle) = gripper.attach_joint.take()
	{
		physics.impulse_joint_set.remove(joint_handle, true);
	};
	if let Some(joint_handle) = gripper.spring_joint.take()
	{
		physics.impulse_joint_set.remove(joint_handle, true);
	};

	let mut parent_query = world.query_one::<&comps::Physics>(gripper.parent).unwrap();
	let parent_physics = parent_query.get().unwrap();
	let parent_body = physics.rigid_body_set.get(parent_physics.handle).unwrap();

	let desired_pos = parent_body.rotation() * gripper.offset + parent_body.translation();
	let desired_rot = *parent_body.rotation();
	let gripper_body = physics
		.rigid_body_set
		.get_mut(gripper_physics.handle)
		.unwrap();
	for collider_handle in gripper_body.colliders()
	{
		let collider = physics.collider_set.get_mut(*collider_handle).unwrap();
		collider.set_collision_groups(InteractionGroups::new(NO_COLLISION, Group::empty()));
	}
	gripper_body.set_translation(desired_pos, true);
	gripper_position.pos = Point3::origin() + desired_pos;
	gripper_position.rot = desired_rot;
	gripper_position.snapshot();

	let joint = FixedJointBuilder::new()
		.local_anchor1(gripper.offset.into())
		.local_anchor2(Point3::origin());
	let joint_handle = physics.impulse_joint_set.insert(
		parent_physics.handle,
		gripper_physics.handle,
		joint,
		true,
	);
	gripper.attach_joint = Some(joint_handle);
}

pub fn spawn_player(
	pos: Point3<f32>, physics: &mut Physics, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene_name = "data/test.glb";
	game_state::cache_scene(state, scene_name)?;
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Controller::new(),
		comps::Scene::new(scene_name),
		comps::Light {
			color: Color::from_rgb_f(1., 1., 1.),
			intensity: 300.,
			static_: false,
		},
	));
	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.angular_damping(10.)
		.linear_damping(5.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.5)
		.restitution(0.1)
		//.mass(4.0)
		.mass_properties(MassProperties::new(
			Point3::origin(),
			4.0,
			1000.0f32 * Vector3::new(1., 1., 1.),
		))
		.friction(0.)
		.user_data(entity.to_bits().get() as u128)
		.collision_groups(InteractionGroups::new(PLAYER_GROUP, PLAYER_GROUP | BIG_GROUP | SMALL_GROUP))
		//.active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
		.build();
	let ball_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		ball_body_handle,
		&mut physics.rigid_body_set,
	);
	world.insert_one(entity, comps::Physics::new(ball_body_handle))?;

	let gripper_1 = spawn_gripper(
		pos,
		Vector3::new(-0.5, -0.3, -0.5),
		entity,
		physics,
		world,
		state,
	)?;
	let gripper_2 = spawn_gripper(
		pos,
		Vector3::new(0.5, -0.3, -0.5),
		entity,
		physics,
		world,
		state,
	)?;

	attach_gripper_to_parent(gripper_1, world, physics);
	attach_gripper_to_parent(gripper_2, world, physics);
	world.insert_one(entity, comps::Grippers::new([gripper_1, gripper_2]))?;

	Ok(entity)
}

pub fn spawn_gripper(
	parent_pos: Point3<f32>, offset: Vector3<f32>, parent: hecs::Entity, physics: &mut Physics,
	world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let pos = parent_pos + offset;
	let scene_name = "data/gripper.glb";
	game_state::cache_scene(state, scene_name)?;
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Scene::new(scene_name),
		comps::Gripper::new(parent, offset),
		comps::Light {
			color: Color::from_rgb_f(0., 1., 0.),
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
		.mass(0.1)
		.user_data(entity.to_bits().get() as u128)
		.active_events(ActiveEvents::COLLISION_EVENTS)
		.build();
	let ball_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		ball_body_handle,
		&mut physics.rigid_body_set,
	);

	world.insert_one(entity, comps::Physics::new(ball_body_handle))?;
	Ok(entity)
}

pub fn spawn_connector(
	start: hecs::Entity, end: hecs::Entity, start_offset: Vector3<f32>, end_offset: Vector3<f32>,
	world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene_name = "data/connector.glb";
	game_state::cache_scene(state, scene_name)?;
	let mut scene = comps::AdditiveScene::new(scene_name);
	scene.color = Color::from_rgb_f(0.0, 0.5, 0.0);
	let entity = world.spawn((
		comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
		comps::Connector::new(start, end, start_offset, end_offset),
		scene,
	));
	Ok(entity)
}

pub fn spawn_hit(
	pos: Point3<f32>, rot: UnitQuaternion<f32>, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene_name = "data/hit.glb";
	let mut scene = comps::AdditiveScene::new(scene_name);
	scene.color = Color::from_rgb_f(0.5, 0.5, 0.);
	game_state::cache_scene(state, scene_name)?;
	let entity = world.spawn((
		comps::Position::new_scaled(pos, rot, Vector3::from_element(1.)),
		scene,
		comps::ExplosionScaling::new(state.hs.time() + 0.2),
	));
	Ok(entity)
}

pub fn spawn_bullet(
	pos: Point3<f32>, rot: UnitQuaternion<f32>, vel: Vector3<f32>, parent: hecs::Entity,
	physics: &mut Physics, world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene_name = "data/gripper.glb";
	game_state::cache_scene(state, scene_name)?;
	let entity = world.spawn((
		comps::Position::new(pos, rot),
		comps::Scene::new(scene_name),
		comps::OnCollideEfffects::new(&[
			comps::Effect::Die,
			comps::Effect::Damage {
				amount: 10.,
				owner: parent,
			},
			comps::Effect::SpawnHit,
		]),
		comps::Light {
			color: Color::from_rgb_f(1., 1., 0.),
			intensity: 100.,
			static_: false,
		},
	));
	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.rotation(rot.scaled_axis())
		.angular_damping(1.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.2)
		.restitution(0.8)
		.mass(0.1)
		.user_data(entity.to_bits().get() as u128)
		.active_events(ActiveEvents::COLLISION_EVENTS)
		.collision_groups(InteractionGroups::new(
			SMALL_GROUP,
			PLAYER_GROUP | BIG_GROUP,
		))
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

	world.insert_one(entity, comps::Physics::new(ball_body_handle))?;
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
		comps::Scene::new(scene_name),
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
	world.insert_one(entity, comps::Physics::new(rigid_body_handle))?;
	Ok(entity)
}

struct Map
{
	world: hecs::World,
	physics: Physics,
	camera_target: comps::Position,
	player: hecs::Entity,
	level: hecs::Entity,
	delayed_effects: Vec<(comps::Effect, hecs::Entity, hecs::Entity)>,
}

impl Map
{
	fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		let mut world = hecs::World::new();

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
		spawn_robot(Point3::new(2.5, 1.5, 0.), &mut physics, &mut world, state)?;
		let player = spawn_player(Point3::new(2., 2.5, 4.), &mut physics, &mut world, state)?;
		let level = spawn_level("data/test_level.glb", state, &mut physics, &mut world)?;

		Ok(Self {
			world: world,
			physics: physics,
			camera_target: comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
			player: player,
			level: level,
			delayed_effects: vec![],
		})
	}

	fn logic(&mut self, state: &mut game_state::GameState)
	-> Result<Option<game_state::NextScreen>>
	{
		let mut to_die = vec![];
		let mut effects = vec![];
		let mut spawn_fns: Vec<
			Box<dyn FnOnce(&mut Map, &mut game_state::GameState) -> Result<hecs::Entity>>,
		> = vec![];
		std::mem::swap(&mut self.delayed_effects, &mut effects);

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
				state.controls.clear_action_state(*action); // TODO: Rework
			}

			self.camera_target.pos = position.pos;
			self.camera_target.rot = position.rot;
			self.camera_target.scale = position.scale;
		}
		if state.controls.get_action_state(game_state::Action::Pause) > 0.5
		{
			state.controls.clear_action_state(game_state::Action::Pause);
			state.hs.paused = !state.hs.paused;
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
					controller.want_fire = false;
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
							else
							{
								controller.want_fire = true;
							}
						}
					}
					else
					{
						new_state = Some(comps::AIState::Idle);
					}
				}
			}
			if let Some(new_state) = new_state
			{
				ai.state = new_state;
			}
		}

		// ExplosionScaling.
		for (id, (position, explosion_scaling)) in self
			.world
			.query::<(&mut comps::Position, &comps::ExplosionScaling)>()
			.iter()
		{
			if state.hs.time() > explosion_scaling.time_to_die
			{
				to_die.push(id);
			}
			position.scale += Vector3::from_element(4. * DT);
		}

		// Friction + force resetting.
		for (_, (_position, physics)) in self
			.world
			.query::<(&comps::Position, &mut comps::Physics)>()
			.iter()
		{
			//let f = 0.9;
			let body = &mut self.physics.rigid_body_set[physics.handle];
			body.reset_forces(true);
			body.reset_torques(true);
			//body.add_force(-f * body.velocity_at_point(&position.pos), true);
			//let f = 0.5;
			//dbg!(body.angvel());
			//dbg!(body.translation());
			//body.add_torque(-f * body.angvel(), true);
		}

		// Movement.
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

		// Weapons.
		for (id, (controller, weapon, position)) in self
			.world
			.query::<(&comps::Controller, &mut comps::Weapon, &comps::Position)>()
			.iter()
		{
			if controller.want_fire && state.hs.time > weapon.time_to_fire
			{
				weapon.time_to_fire = state.hs.time + weapon.fire_delay;
				let (forward, _, _) = get_dirs(position.rot);
				let bullet_pos = position.pos + position.rot * weapon.offset;
				let bullet_rot = position.rot;
				spawn_fns.push(Box::new(move |map, state| -> Result<hecs::Entity> {
					spawn_bullet(
						bullet_pos,
						bullet_rot,
						forward,
						id,
						&mut map.physics,
						&mut map.world,
						state,
					)
				}));
			}
		}

		// Physics.
		let handler = PhysicsEventHandler::new();
		self.physics.step(&handler);

		for (event, _contact_pair) in handler.collision_events.try_read().unwrap().iter()
		{
			if let CollisionEvent::Started(collider_handle_1, collider_handle_2, _) = event
			{
				for (collider_handle, other_collider_handle) in [
					(*collider_handle_1, *collider_handle_2),
					(*collider_handle_2, *collider_handle_1),
				]
				{
					let collider = self.physics.collider_set.get(collider_handle).unwrap();
					let other_collider = self
						.physics
						.collider_set
						.get(other_collider_handle)
						.unwrap();
					let body_handle = collider.parent().unwrap();
					let id = hecs::Entity::from_bits(collider.user_data as u64).unwrap();
					let other_id =
						hecs::Entity::from_bits(other_collider.user_data as u64).unwrap();

					if let Ok(on_collide_effects) = self.world.get::<&comps::OnCollideEfffects>(id)
					{
						for effect in &on_collide_effects.effects
						{
							effects.push((effect.clone(), id, other_id));
						}
					}

					if let Some(gripper) = self
						.world
						.query_one::<&mut comps::Gripper>(id)
						.unwrap()
						.get()
					{
						if gripper.status != comps::GripperStatus::Flying
						{
							continue;
						}
						let gripper_vel = {
							self.world
								.query_one::<&mut comps::Physics>(id)
								.unwrap()
								.get()
								.unwrap()
								.old_vel
						};

						let body = self.physics.rigid_body_set.get_mut(body_handle).unwrap();
						let gripper_pos = *body.translation();
						if let Some(parent_physics) = self
							.world
							.query_one::<&mut comps::Physics>(gripper.parent)
							.unwrap()
							.get()
						{
							if other_id == self.level
							{
								if let Some(joint_handle) = gripper.attach_joint.take()
								{
									self.physics.impulse_joint_set.remove(joint_handle, true);
								};
								if let Some(joint_handle) = gripper.spring_joint.take()
								{
									self.physics.impulse_joint_set.remove(joint_handle, true);
								};

								let joint = FixedJointBuilder::new()
									.local_anchor1(Point3::new(0., 0., 0.))
									.local_anchor2(Point3::origin() + body.translation());
								let joint_handle = self.physics.impulse_joint_set.insert(
									body_handle,
									other_collider.parent().unwrap(),
									joint,
									true,
								);
								gripper.attach_joint = Some(joint_handle);

								let joint = SpringJointBuilder::new(0.1, 30., 10.)
									.local_anchor1(Point3::origin())
									.local_anchor2(Point3::origin());
								let joint_handle = self.physics.impulse_joint_set.insert(
									body_handle,
									parent_physics.handle,
									joint,
									true,
								);
								gripper.spring_joint = Some(joint_handle);
								gripper.status = comps::GripperStatus::AttachedToLevel;
							}
							else
							{
								if let Some(target_physics) = self
									.world
									.query_one::<&comps::Physics>(other_id)
									.unwrap()
									.get()
								{
									let target_body = self
										.physics
										.rigid_body_set
										.get(target_physics.handle)
										.unwrap();
									let dir = (target_body.translation() - gripper_pos).normalize();
									let rel_vel = (gripper_vel - target_physics.old_vel).dot(&dir);
									effects.push((
										comps::Effect::Damage {
											amount: rel_vel.max(0.),
											owner: gripper.parent,
										},
										id,
										other_id,
									));
									effects.push((comps::Effect::SpawnHit, id, other_id));
									self.delayed_effects.push((
										comps::Effect::GripperPierce {
											old_vel: 0.75 * gripper_vel,
										},
										id,
										other_id,
									));
								}
							}
						}
					}
				}
			}
		}

		// Grippers.
		// Do these after physics so we can compute the velocity correctly.
		let mut want_grip = vec![];
		for (id, (controller, position, grippers)) in self
			.world
			.query::<(
				&mut comps::Controller,
				&comps::Position,
				&mut comps::Grippers,
			)>()
			.iter()
		{
			for (want_gripper, gripper_id) in
				itertools::izip!(&controller.want_gripper, &mut grippers.grippers,)
			{
				if *want_gripper
				{
					want_grip.push((id, *position, *gripper_id));
				}
			}
		}

		for (id, parent_position, gripper_id) in want_grip
		{
			let mut attach = false;
			let gripper_offset;
			{
				let mut gripper_query = self
					.world
					.query_one::<(&comps::Physics, &mut comps::Gripper)>(gripper_id)
					.unwrap();
				let (gripper_physics, gripper) = gripper_query.get().unwrap();
				let (forward, _, _) = get_dirs(parent_position.rot);
				gripper_offset = gripper.offset;
				match gripper.status
				{
					comps::GripperStatus::AttachedToParent =>
					{
						if state.hs.time() >= gripper.time_to_grip
						{
							let gripper_body = self
								.physics
								.rigid_body_set
								.get_mut(gripper_physics.handle)
								.unwrap();
							gripper_body.apply_impulse(forward, true);
							for collider_handle in gripper_body.colliders()
							{
								let collider =
									self.physics.collider_set.get_mut(*collider_handle).unwrap();
								collider.set_collision_groups(InteractionGroups::new(
									GRIPPER_GROUP,
									BIG_GROUP,
								));
							}
							if let Some(joint_handle) = gripper.attach_joint.take()
							{
								self.physics.impulse_joint_set.remove(joint_handle, true);
							};
							gripper.time_to_grip = state.hs.time() + 0.1;
							gripper.status = comps::GripperStatus::Flying;
							spawn_fns.push(Box::new(move |map, state| -> Result<hecs::Entity> {
								let connector = spawn_connector(
									id,
									gripper_id,
									gripper_offset,
									Vector3::zeros(),
									&mut map.world,
									state,
								)?;
								map.world
									.get::<&mut comps::Gripper>(gripper_id)
									.unwrap()
									.connector = Some(connector);
								Ok(connector)
							}));
						}
					}
					comps::GripperStatus::Flying | comps::GripperStatus::AttachedToLevel =>
					{
						gripper.status = comps::GripperStatus::AttachedToParent;
						if let Some(connector_id) = gripper.connector.take()
						{
							to_die.push(connector_id);
						}
						attach = true;
					}
				}
			}
			if attach
			{
				attach_gripper_to_parent(gripper_id, &self.world, &mut self.physics);
			}
		}

		// Physics -> position sync.
		for (_id, (position, physics)) in self
			.world
			.query::<(&mut comps::Position, &mut comps::Physics)>()
			.iter()
		{
			let body = &self.physics.rigid_body_set[physics.handle];
			position.pos = Point3::from(*body.translation());
			position.rot = *body.rotation();
			physics.old_vel = *body.linvel();
		}

		// Spawn fns;
		for spawn_fn in spawn_fns
		{
			spawn_fn(self, state)?;
		}

		// Connector upkeep
		let mut connector_ends = vec![];
		for (id, (connector, _)) in self
			.world
			.query::<(&comps::Connector, &mut comps::Position)>()
			.iter()
		{
			if let (Ok(start_position), Ok(end_position)) = (
				self.world.get::<&comps::Position>(connector.start),
				self.world.get::<&comps::Position>(connector.end),
			)
			{
				connector_ends.push((start_position.clone(), end_position.clone()))
			}
			else
			{
				to_die.push(id);
			}
		}

		// Is this valid?
		for ((_, (connector, position)), (start_position, end_position)) in itertools::izip!(
			self.world
				.query::<(&comps::Connector, &mut comps::Position)>()
				.iter(),
			connector_ends
		)
		{
			let start_pos = start_position.pos + start_position.rot * connector.start_offset;
			let end_pos = end_position.pos + end_position.rot * connector.end_offset;

			let dir = (end_pos - start_pos).normalize();
			let rot = UnitQuaternion::from_axis_angle(
				&Unit::new_unchecked(dir),
				5. * state.hs.time() as f32,
			);
			let new_pos = ((start_pos.coords + end_pos.coords) / 2.0).into();
			let new_rot = rot * UnitQuaternion::face_towards(&dir, &Vector3::y());
			let new_scale = Vector3::new(1., 1., (start_pos - end_pos).norm() / 2.);

			position.pos = new_pos;
			position.rot = new_rot;
			position.scale = new_scale;
		}

		while !effects.is_empty()
		{
			let new_effects = vec![];
			// Effects.
			for (effect, id, other_id) in effects.drain(..)
			{
				match effect
				{
					comps::Effect::Die =>
					{
						to_die.push(id);
					}
					comps::Effect::Damage { amount, owner } =>
					{
						if let Ok(mut health) = self.world.get::<&mut comps::Health>(other_id)
						{
							health.health -= amount;
						}
						if let Ok(mut ai) = self.world.get::<&mut comps::AI>(other_id)
						{
							ai.state = comps::AIState::Attacking(owner);
						}
					}
					comps::Effect::GripperPierce { old_vel } =>
					{
						if self.world.contains(other_id)
						{
							let mut attach_gripper = false;
							if let Ok(mut gripper) = self.world.get::<&mut comps::Gripper>(id)
							{
								attach_gripper = true;
								gripper.status = comps::GripperStatus::AttachedToParent;
								if let Some(connector_id) = gripper.connector.take()
								{
									to_die.push(connector_id);
								}
							}
							if attach_gripper
							{
								attach_gripper_to_parent(id, &self.world, &mut self.physics);
							}
						}
						else
						{
							if let Ok(physics) = self.world.get::<&comps::Physics>(id)
							{
								let body =
									self.physics.rigid_body_set.get_mut(physics.handle).unwrap();
								body.set_linvel(old_vel, true);
							}
						}
					}
					comps::Effect::SpawnHit =>
					{
						let mut src_pos = None;
						if let Ok(position) = self.world.get::<&comps::Position>(id)
						{
							src_pos = Some(position.pos);
						}
						let mut dest_pos = None;
						if let Ok(position) = self.world.get::<&comps::Position>(other_id)
						{
							dest_pos = Some(position.pos);
						}
						if let (Some(src_pos), Some(dest_pos)) = (src_pos, dest_pos)
						{
							let dir = (dest_pos - src_pos).normalize();
							let rot = UnitQuaternion::face_towards(&dir, &Vector3::y());
							spawn_hit(src_pos, rot, &mut self.world, state)?;
						}
					}
				}
			}

			// Health.
			for (id, health) in self.world.query::<&mut comps::Health>().iter()
			{
				if health.health <= 0.0
				{
					to_die.push(id);
				}
			}
			effects = new_effects;
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
		//let (forward, _, up) = get_dirs(self.camera_target.draw_rot(alpha));
		Isometry3 {
			rotation: self.camera_target.draw_rot(alpha),
			translation: self.camera_target.draw_pos(alpha).coords.into(),
			//translation: (self.camera_target.draw_pos(alpha).coords - 2. * forward + 0.5 * up).into(),
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
			let scale = Matrix4::new_nonuniform_scaling(&position.draw_scale(alpha));

			state.hs.core.use_transform(&utils::mat4_to_transform(
				camera.to_homogeneous() * shift * scale,
			));
			state
				.hs
				.core
				.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift * scale))
				.ok();
			// THIS IS HORRIBLE
			let color = scene.color;
			let (r, g, b, a) = color.to_rgba_f();
			let color = [r, g, b, a];
			state
				.hs
				.core
				.set_shader_uniform("base_color", &[color][..])
				.ok();

			state.get_scene(&scene.scene).unwrap().draw(
				&state.hs.core,
				&state.hs.prim,
				material_mapper,
			);
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

		// TODO: NO WAY!
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
			state.get_bitmap(texture_name).into_slhack()
		};
		state
			.hs
			.core
			.use_shader(Some(state.forward2_shader.as_ref().unwrap()))
			.unwrap();
		state
			.hs
			.core
			.use_projection_transform(&utils::mat4_to_transform(project.to_homogeneous()));
		state.hs.core.set_depth_test(Some(DepthFunction::Less));
		state
			.hs
			.core
			.set_blender(BlendOperation::Add, BlendMode::One, BlendMode::One);

		// Second forward pass, for additive stuff.
		for (_, (position, scene)) in self
			.world
			.query::<(&comps::Position, &comps::AdditiveScene)>()
			.iter()
		{
			let shift = Isometry3 {
				translation: position.draw_pos(alpha).coords.into(),
				rotation: position.draw_rot(alpha),
			}
			.to_homogeneous();
			let scale = Matrix4::new_nonuniform_scaling(&position.draw_scale(alpha));

			state.hs.core.use_transform(&utils::mat4_to_transform(
				camera.to_homogeneous() * shift * scale,
			));

			// THIS IS HORRIBLE
			let color = scene.color;
			let (r, g, b, a) = color.to_rgba_f();
			let color = [r, g, b, a];
			state
				.hs
				.core
				.set_shader_uniform("base_color", &[color][..])
				.ok();

			state.get_scene(&scene.scene).unwrap().draw(
				&state.hs.core,
				&state.hs.prim,
				material_mapper,
			);
		}

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
