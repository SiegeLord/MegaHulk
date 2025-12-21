use crate::error::{Result, ResultHelper};
use crate::game_state::DT;
use crate::{components as comps, game_state, ui, utils};
use allegro::*;
use allegro_font::*;
use allegro_primitives::*;
use allegro_sys::*;
use itertools::Itertools;
use na::{
	Isometry3, Matrix4, Perspective3, Point2, Point3, Quaternion, RealField, Rotation2, Rotation3,
	Similarity3, Transform3, Unit, UnitQuaternion, Vector2, Vector3, Vector4,
};
use nalgebra as na;
use rand::prelude::*;
use rand_distr::Distribution;
use rapier3d::dynamics::{
	CCDSolver, FixedJointBuilder, ImpulseJointHandle, ImpulseJointSet, IntegrationParameters,
	IslandManager, MassProperties, MultibodyJointSet, RigidBody, RigidBodyBuilder, RigidBodyHandle,
	RigidBodySet, SpringJointBuilder,
};
use rapier3d::geometry::{
	ColliderBuilder, ColliderHandle, ColliderSet, CollisionEvent, ContactPair, DefaultBroadPhase,
	Group, InteractionGroups, NarrowPhase, Ray, SharedShape, TriMeshFlags,
};
use rapier3d::pipeline::{ActiveEvents, EventHandler, PhysicsPipeline, QueryFilter};
use serde_derive::{Deserialize, Serialize};
use slhack::{controls, scene, sprite, ui as slhack_ui};

use std::collections::HashMap;
use std::f32::consts::PI;
use std::sync::RwLock;

const PLAYER_GROUP: Group = Group::GROUP_1;
const GRIPPER_GROUP: Group = Group::GROUP_2;
const BIG_GROUP: Group = Group::GROUP_3;
const SMALL_GROUP: Group = Group::GROUP_4;

const POWER_LEVEL_TIME: f64 = 10.0;
const ENERGY_AMOUNT: f32 = 20.;

fn color_to_array(color: Color) -> [f32; 4]
{
	// TODO: Put this into RustAllegro.
	let (r, g, b, a) = color.to_rgba_f();
	[r, g, b, a]
}

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

	fn ray_cast(
		&self, pos: Point3<f32>, dir: Vector3<f32>, source: RigidBodyHandle,
	) -> Option<(ColliderHandle, f32)>
	{
		let query_pipeline = self.broad_phase.as_query_pipeline(
			self.narrow_phase.query_dispatcher(),
			&self.rigid_body_set,
			&self.collider_set,
			QueryFilter::default()
				.exclude_rigid_body(source)
				.groups(InteractionGroups::new(BIG_GROUP, BIG_GROUP | PLAYER_GROUP)),
		);

		let ray = Ray::new(pos, dir);
		query_pipeline.cast_ray(&ray, f32::MAX, true)
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
			map: Map::new("data/test_level.cfg", state)?,
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
	pos: Point3<f32>, rot: UnitQuaternion<f32>, physics: &mut Physics, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene_name = "data/robot1.glb";
	let scene = game_state::cache_scene(state, scene_name)?;

	let entity = world.spawn((
		comps::Position::new(pos, rot),
		comps::Scene::new(scene_name),
		comps::Controller::new(),
		comps::Health::new(5.),
		comps::AI::new(),
		comps::Weapon::new(Vector3::new(0., 0., -1.)),
		comps::Stats::new(comps::StatValues::new_robot()),
		comps::OnDeathEffects::new(&[
			comps::Effect::SpawnExplosion {
				kind: comps::ExplosionKind::Big,
			},
			comps::Effect::SpawnItem {
				spawn_table: vec![(0.5, comps::ItemKind::Energy)],
			},
		]),
	));

	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.rotation(rot.scaled_axis())
		.angular_damping(10.)
		.linear_damping(5.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let body_handle = physics.rigid_body_set.insert(rigid_body);

	for (vertices, _) in get_collision_trimeshes(scene)
	{
		let collider = ColliderBuilder::convex_hull(&vertices).ok_or_else(|| format!("Couldn't create convex hull for {}", scene_name))?
			.restitution(0.1)
			.density(1.0)
			.friction(0.)
			.user_data(entity.to_bits().get() as u128)
			.collision_groups(InteractionGroups::new(BIG_GROUP, PLAYER_GROUP | BIG_GROUP | SMALL_GROUP | GRIPPER_GROUP))
			//.active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
			.build();
		physics
			.collider_set
			.insert_with_parent(collider, body_handle, &mut physics.rigid_body_set);
	}
	world.insert_one(entity, comps::Physics::new(body_handle))?;
	Ok(entity)
}

pub fn spawn_reactor(
	pos: Point3<f32>, rot: UnitQuaternion<f32>, physics: &mut Physics, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene_name = "data/robot1.glb";
	let scene = game_state::cache_scene(state, scene_name)?;

	let mut health = comps::Health::new(35.);
	health.remove_on_death = false;
	health.death_effects = vec![
		comps::Effect::ExplosionSpawner {
			kind: comps::ExplosionKind::Small,
		},
		comps::Effect::OpenExit,
	];

	let entity = world.spawn((
		comps::Position::new(pos, rot),
		comps::Scene::new(scene_name),
		health,
	));

	let rigid_body = RigidBodyBuilder::fixed()
		.translation(pos.coords)
		.rotation(rot.scaled_axis())
		.user_data(entity.to_bits().get() as u128)
		.build();
	let body_handle = physics.rigid_body_set.insert(rigid_body);

	for (vertices, _) in get_collision_trimeshes(scene)
	{
		let collider = ColliderBuilder::convex_hull(&vertices).ok_or_else(|| format!("Couldn't create convex hull for {}", scene_name))?
			.restitution(0.1)
			.density(1.0)
			.friction(0.)
			.user_data(entity.to_bits().get() as u128)
			.collision_groups(InteractionGroups::new(BIG_GROUP, PLAYER_GROUP | BIG_GROUP | SMALL_GROUP | GRIPPER_GROUP))
			//.active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
			.build();
		physics
			.collider_set
			.insert_with_parent(collider, body_handle, &mut physics.rigid_body_set);
	}
	world.insert_one(entity, comps::Physics::new(body_handle))?;
	Ok(entity)
}

pub fn spawn_player_exit_track(
	scene_name: &str, obj_name: &str, target: hecs::Entity, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene = state.get_scene(scene_name)?;
	let obj_idx = scene
		.objects
		.iter()
		.position(|obj| obj.name.starts_with(obj_name));

	let obj_idx =
		obj_idx.ok_or_else(|| format!("Could not find {} in {}", obj_name, scene_name))?;

	let entity = world.spawn((
		comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
		comps::SceneObjectPosition::new(
			scene_name,
			obj_idx as i32,
			scene::AnimationState::new("Play", true),
		),
		comps::PositionCopier { target },
	));
	Ok(entity)
}

pub fn spawn_exit_explosions(
	scene_name: &str, obj_name: &str, world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene = state.get_scene(scene_name)?;
	let obj_idx = scene
		.objects
		.iter()
		.position(|obj| obj.name.starts_with(obj_name));

	let obj_idx =
		obj_idx.ok_or_else(|| format!("Could not find {} in {}", obj_name, scene_name))?;

	let entity = world.spawn((
		comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
		comps::SceneObjectPosition::new(
			scene_name,
			obj_idx as i32,
			scene::AnimationState::new("Play", true),
		),
		comps::ExplosionSpawner::new(comps::ExplosionKind::Big),
	));
	Ok(entity)
}

pub fn spawn_animated_target(
	scene_name: &str, obj_name: &str, world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene = state.get_scene(scene_name)?;
	let obj_idx = scene
		.objects
		.iter()
		.position(|obj| obj.name.starts_with(obj_name));

	let obj_idx =
		obj_idx.ok_or_else(|| format!("Could not find {} in {}", obj_name, scene_name))?;

	let entity = world.spawn((
		comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
		comps::SceneObjectPosition::new(
			scene_name,
			obj_idx as i32,
			scene::AnimationState::new("Play", true),
		),
	));
	Ok(entity)
}

pub fn spawn_death_camera(
	pos: Point3<f32>, target: hecs::Entity, physics: &mut Physics, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::FaceTowards::new(target),
	));
	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.linear_damping(2.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.5)
		.restitution(0.1)
		.mass(0.1)
		.friction(0.)
		.user_data(entity.to_bits().get() as u128)
		.collision_groups(InteractionGroups::new(
			SMALL_GROUP,
			PLAYER_GROUP | BIG_GROUP,
		))
		.build();
	let body_handle = physics.rigid_body_set.insert(rigid_body);
	physics
		.collider_set
		.insert_with_parent(collider, body_handle, &mut physics.rigid_body_set);
	let mut camera_physics = comps::Physics::new(body_handle);
	let vel = 2.
		* Vector3::<f64>::from_row_slice(&rand_distr::UnitSphere.sample(&mut rand::rng()))
			.cast::<f32>();
	physics
		.rigid_body_set
		.get_mut(body_handle)
		.unwrap()
		.apply_impulse(vel, true);
	camera_physics.copy_rot = false;
	world.insert_one(entity, camera_physics)?;
	Ok(entity)
}

pub fn spawn_exit_trigger(
	pos: Point3<f32>, rot: UnitQuaternion<f32>, physics: &mut Physics, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let entity = world.spawn((
		comps::Position::new(pos, rot),
		comps::OnCollideEffects::new(&[comps::Effect::StartExitAnimation]),
	));

	let rigid_body = RigidBodyBuilder::fixed()
		.translation(pos.coords)
		.rotation(rot.scaled_axis())
		.user_data(entity.to_bits().get() as u128)
		.build();
	let body_handle = physics.rigid_body_set.insert(rigid_body);

	let collider = ColliderBuilder::ball(1.)
		.sensor(true)
		.user_data(entity.to_bits().get() as u128)
		.collision_groups(InteractionGroups::new(PLAYER_GROUP, PLAYER_GROUP))
		.active_events(ActiveEvents::COLLISION_EVENTS)
		.build();
	physics
		.collider_set
		.insert_with_parent(collider, body_handle, &mut physics.rigid_body_set);
	world.insert_one(entity, comps::Physics::new(body_handle))?;
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
		collider.set_collision_groups(InteractionGroups::none());
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

pub fn spawn_item(
	pos: Point3<f32>, vel: Vector3<f32>, kind: comps::ItemKind, physics: &mut Physics,
	world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let (scene_name, scene_color, light_color, map_scene, effects) = match kind
	{
		comps::ItemKind::Energy => (
			"data/energy.glb",
			Color::from_rgb_f(1., 1., 1.),
			Some(Color::from_rgb_f(0., 0., 1.)),
			None,
			vec![comps::Effect::PickupItem { kind: kind }],
		),
		comps::ItemKind::Key { kind: key_kind } => (
			"data/key.glb",
			key_kind.color(),
			Some(key_kind.color()),
			None,
			vec![comps::Effect::PickupItem { kind: kind }],
		),
		comps::ItemKind::Gift => (
			"data/gift.glb",
			Color::from_rgb_f(1., 1., 1.),
			None,
			Some(("data/gift_map.glb", Color::from_rgb_f(1., 0., 1.))),
			vec![comps::Effect::PickupItem { kind: kind }],
		),
	};
	game_state::cache_scene(state, scene_name)?;
	let dir = Vector3::from_row_slice(&rand_distr::UnitSphere.sample(&mut rand::rng()));
	let rot = UnitQuaternion::face_towards(&dir, &Vector3::y());

	let mut scene = comps::Scene::new_with_animation(scene_name, "Play");
	scene.color = scene_color;

	let entity = world.spawn((
		comps::Position::new(pos, rot),
		scene,
		comps::OnCollideEffects::new(&effects),
	));
	if let Some((map_scene_name, map_scene_color)) = map_scene
	{
		game_state::cache_scene(state, map_scene_name)?;
		let mut map_scene = comps::MapScene::new(map_scene_name);
		map_scene.color = map_scene_color;
		map_scene.explored = true;
		world.insert_one(entity, map_scene)?;
	}
	if let Some(light_color) = light_color
	{
		world.insert_one(
			entity,
			comps::Light {
				color: light_color,
				intensity: 100.,
				static_: false,
			},
		)?;
	}
	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.angular_damping(10.)
		.linear_damping(3.)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.25)
		.restitution(0.1)
		.mass(0.1)
		.friction(0.)
		.user_data(entity.to_bits().get() as u128)
		.collision_groups(InteractionGroups::new(BIG_GROUP, PLAYER_GROUP | BIG_GROUP))
		.active_events(ActiveEvents::COLLISION_EVENTS)
		.build();
	let body_handle = physics.rigid_body_set.insert(rigid_body);
	physics
		.collider_set
		.insert_with_parent(collider, body_handle, &mut physics.rigid_body_set);
	world.insert_one(entity, comps::Physics::new(body_handle))?;

	physics
		.rigid_body_set
		.get_mut(body_handle)
		.unwrap()
		.apply_impulse(vel, true);

	Ok(entity)
}

pub fn spawn_player(
	pos: Point3<f32>, rot: UnitQuaternion<f32>, physics: &mut Physics, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene_name = "data/test.glb";
	game_state::cache_scene(state, scene_name)?;
	let mut map_scene = comps::MapScene::new(scene_name);
	map_scene.explored = true;

	let mut health = comps::Health::new(100.);
	health.remove_on_death = false;
	health.death_effects = vec![
		comps::Effect::SpawnDeathCamera,
		comps::Effect::ExplosionSpawner {
			kind: comps::ExplosionKind::Small,
		},
		comps::Effect::DelayedDeath { delay: 3. },
	];

	let entity = world.spawn((
		comps::Position::new(pos, rot),
		comps::Controller::new(),
		comps::Scene::new(scene_name),
		comps::Stats::new(comps::StatValues::new_player()),
		comps::OnDeathEffects::new(&[comps::Effect::SpawnExplosion {
			kind: comps::ExplosionKind::Big,
		}]),
		health,
		map_scene,
		comps::Inventory::new(),
	));
	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.rotation(rot.scaled_axis())
		.angular_damping(10.)
		.linear_damping(3.)
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

	let mut position = comps::Position::new(Point3::origin(), UnitQuaternion::identity());
	let connector = comps::Connector::new(start, end, start_offset, end_offset);

	let start_position = *world.get::<&comps::Position>(start)?;
	let end_position = *world.get::<&comps::Position>(end)?;

	set_connector_position(
		&connector,
		&mut position,
		&start_position,
		&end_position,
		state,
	);

	let entity = world.spawn((position, connector, scene));
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
		comps::Position::new_scaled(pos, rot, Vector3::from_element(0.1)),
		scene,
		comps::Light {
			color: Color::from_rgb_f(0.5, 0.5, 0.),
			intensity: 500.,
			static_: false,
		},
		comps::ExplosionScaling::new(4.),
		comps::TimeToDie::new(state.hs.time() + 0.2),
	));
	Ok(entity)
}

pub fn spawn_explosion(
	pos: Point3<f32>, kind: comps::ExplosionKind, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let (color, scale) = match kind
	{
		comps::ExplosionKind::Small => (Color::from_rgb_f(1., 0.5, 0.), 0.1),
		comps::ExplosionKind::Big => (Color::from_rgb_f(1., 0.5, 0.), 0.5),
		comps::ExplosionKind::Energy => (Color::from_rgb_f(0.5, 0.5, 1.), 0.2),
	};
	let scene_name = "data/explosion.glb";
	let mut scene = comps::AdditiveScene::new(scene_name);
	scene.color = color;
	game_state::cache_scene(state, scene_name)?;
	let dir = Vector3::from_row_slice(&rand_distr::UnitSphere.sample(&mut rand::rng()));
	let rot = UnitQuaternion::face_towards(&dir, &Vector3::y());
	let entity = world.spawn((
		comps::Position::new_scaled(pos, rot, Vector3::from_element(scale)),
		scene,
		comps::ExplosionScaling::new(4.),
		comps::TimeToDie::new(state.hs.time() + 0.2 * scale as f64),
		comps::Light {
			color: color,
			intensity: scale * 500.,
			static_: false,
		},
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
		comps::OnCollideEffects::new(&[
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

fn spawn_door(
	pos: Point3<f32>, rot: UnitQuaternion<f32>, open_on_exit: bool, key: Option<comps::KeyKind>,
	physics: &mut Physics, world: &mut hecs::World, state: &mut game_state::GameState,
) -> Result<hecs::Entity>
{
	let scene_name = "data/door1.glb";
	let map_scene_name = "data/door_map.glb";
	game_state::cache_scene(state, map_scene_name)?;
	let scene = game_state::cache_scene(state, scene_name)?;

	let mut map_scene = comps::MapScene::new(map_scene_name);
	map_scene.color = key
		.map(|k| k.color())
		.unwrap_or(Color::from_rgb_f(1., 1., 1.));

	//scene.animation_states = animation_states;
	let entity = world.spawn((
		comps::Position::new(pos, rot),
		comps::Scene::new(scene_name),
		map_scene,
		comps::Door::new(open_on_exit, key),
		comps::OnCollideEffects::new(&[comps::Effect::Open]),
	));

	let rigid_body = RigidBodyBuilder::fixed()
		.translation(pos.coords)
		.rotation(rot.scaled_axis())
		.user_data(entity.to_bits().get() as u128)
		.build();
	let body_handle = physics.rigid_body_set.insert(rigid_body);

	for (vertices, _) in get_collision_trimeshes(scene)
	{
		let collider = ColliderBuilder::convex_hull(&vertices)
			.ok_or_else(|| format!("Couldn't create convex hull for {}", scene_name))?
			.restitution(0.1)
			.density(1.0)
			.friction(0.)
			.user_data(entity.to_bits().get() as u128)
			.active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
			.build();
		physics
			.collider_set
			.insert_with_parent(collider, body_handle, &mut physics.rigid_body_set);
	}
	world.insert_one(entity, comps::Physics::new(body_handle))?;
	Ok(entity)
}

fn meshes_to_trimesh(
	meshes: &[scene::Mesh<game_state::MaterialKind>], pos: Point3<f32>, rot: UnitQuaternion<f32>,
	scale: Vector3<f32>,
) -> (Vec<Point3<f32>>, Vec<[u32; 3]>)
{
	let mut vertices = vec![];
	let mut indices = vec![];
	let mut index_offset = 0;

	let shift = Isometry3 {
		translation: pos.coords.into(),
		rotation: rot.into(),
	}
	.to_homogeneous();
	let scale = Matrix4::new_nonuniform_scaling(&scale);
	let transform = Transform3::from_matrix_unchecked(shift * scale);

	for mesh in meshes
	{
		for vtx in &mesh.vtxs
		{
			vertices.push(transform * Point3::new(vtx.x, vtx.y, vtx.z));
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
	(vertices, indices)
}

fn get_collision_trimeshes(
	scene: &scene::Scene<game_state::MaterialKind>,
) -> Vec<(Vec<Point3<f32>>, Vec<[u32; 3]>)>
{
	let mut trimeshes = vec![];
	for object in &scene.objects
	{
		match &object.kind
		{
			scene::ObjectKind::CollisionMesh { meshes } =>
			{
				trimeshes.push(meshes_to_trimesh(
					&meshes[..],
					object.pos,
					object.rot,
					object.scale,
				));
			}
			_ => (),
		}
	}
	if !trimeshes.is_empty()
	{
		return trimeshes;
	}
	for object in &scene.objects
	{
		match &object.kind
		{
			scene::ObjectKind::MultiMesh { meshes } =>
			{
				trimeshes.push(meshes_to_trimesh(
					&meshes[..],
					object.pos,
					object.rot,
					object.scale,
				));
			}
			_ => (),
		}
	}
	trimeshes
}

struct LevelMap
{
	object: game_state::Object,
	depth_output: Bitmap,
	exploration_buffer: Bitmap,
	dirty: bool,
}

impl LevelMap
{
	fn new(object: game_state::Object, state: &game_state::GameState) -> Result<Self>
	{
		let buffer_width = (state.hs.buffer_width() / 2.) as i32;
		let buffer_height = (state.hs.buffer_height() / 2.) as i32;

		let old_depth = state.hs.core.get_new_bitmap_depth();
		let old_format = state.hs.core.get_new_bitmap_format();
		state.hs.core.set_new_bitmap_depth(16);
		let depth_output = Bitmap::new(&state.hs.core, buffer_width, buffer_height)
			.map_err(|_| "Couldn't create depth output".to_string())?;
		state.hs.core.set_new_bitmap_depth(old_depth);
		state.hs.core.set_new_bitmap_format(old_format);

		let exploration_buffer = Bitmap::new(&state.hs.core, 256, 256)
			.map_err(|_| "Couldn't create exploration buffer".to_string())?;
		state.hs.core.set_target_bitmap(Some(&exploration_buffer));
		state.hs.core.clear_to_color(Color::from_rgb_f(0., 0., 0.));

		Ok(Self {
			depth_output: depth_output,
			exploration_buffer: exploration_buffer,
			object: object,
			dirty: false,
		})
	}
}

struct LevelProperties
{
	level: hecs::Entity,
	level_map: LevelMap,
	player: hecs::Entity,
	num_gifts: i32,
}

fn spawn_level(
	scene_name: &str, physics: &mut Physics, world: &mut hecs::World,
	state: &mut game_state::GameState,
) -> Result<LevelProperties>
{
	game_state::cache_scene(state, scene_name)?;

	let level_map = state.with_scene(scene_name, |state, level_scene| {
		let obj_idx = level_scene.objects.iter().position(|o| o.name == "Level");
		let obj_idx =
			obj_idx.ok_or_else(|| format!("Level {} has no Level object.", scene_name))?;
		let mut map_object = level_scene.objects[obj_idx].create_clone(
			state.hs.display.as_mut().unwrap(),
			&state.hs.prim,
			false,
		)?;

		if let slhack::scene::ObjectKind::MultiMesh { meshes } = &mut map_object.kind
		{
			for mesh in meshes
			{
				let desc = &mut mesh.material.as_mut().unwrap().desc;
				desc.material_kind = game_state::MaterialKind::LevelMap;
				desc.texture = "data/map_texture.png".to_string();
				let len = mesh.vertex_buffer.len();
				for vtx in &mut mesh.vtxs
				{
					vtx.color = Color::from_rgba_f(0., 0., 0., 0.);
				}
				let mut lock = mesh.vertex_buffer.lock_write_only(0, len).unwrap();
				lock.copy_from_slice(&mesh.vtxs);
			}
		}
		LevelMap::new(map_object, state)
	})?;

	let level_scene = state.get_scene(scene_name)?;
	let entity = world.spawn((
		comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
		comps::Scene::new(scene_name),
	));

	let rigid_body = RigidBodyBuilder::fixed()
		.user_data(entity.to_bits().get() as u128)
		.build();
	let body_handle = physics.rigid_body_set.insert(rigid_body);

	for (vertices, indices) in get_collision_trimeshes(level_scene)
	{
		let collider = ColliderBuilder::trimesh(vertices, indices)?
			.collision_groups(InteractionGroups::new(
				BIG_GROUP,
				PLAYER_GROUP | BIG_GROUP | SMALL_GROUP | GRIPPER_GROUP,
			))
			.user_data(entity.to_bits().get() as u128)
			.build();
		physics
			.collider_set
			.insert_with_parent(collider, body_handle, &mut physics.rigid_body_set);
	}
	world.insert_one(entity, comps::Physics::new(body_handle))?;

	let mut player_spawn_fn = None;
	let mut spawn_fns: Vec<
		Box<
			dyn FnOnce(
				&mut Physics,
				&mut hecs::World,
				&mut game_state::GameState,
			) -> Result<hecs::Entity>,
		>,
	> = vec![];
	let mut num_gifts = 0;
	for object in &level_scene.objects
	{
		let pos = object.pos.clone();
		let rot = object.rot.clone();
		match object.kind
		{
			scene::ObjectKind::Light { color, intensity } =>
			{
				spawn_light(
					object.pos,
					comps::Light {
						color: color,
						intensity: intensity / 30.,
						static_: true,
					},
					world,
				)?;
			}
			scene::ObjectKind::Empty =>
			{
				if object.name == "PlayerStart"
				{
					player_spawn_fn = Some(move |physics, world, state| -> Result<hecs::Entity> {
						spawn_player(pos, rot, physics, world, state)
					});
				}
				else if object.name.starts_with("Door")
				{
					let open_on_exit = object
						.properties
						.as_object()
						.and_then(|o| o.get("open_on_exit"))
						.and_then(|b| b.as_bool())
						.unwrap_or(false);

					let key_str = object
						.properties
						.as_object()
						.and_then(|o| o.get("key"))
						.and_then(|s| s.as_str());

					let key = if let Some(key_str) = key_str
					{
						Some(comps::KeyKind::from_str(key_str).ok_or_else(|| {
							format!(
								"Unknown key type '{}' for Door object: {}",
								key_str, object.name
							)
						})?)
					}
					else
					{
						None
					};

					spawn_fns.push(Box::new(
						move |physics, world, state| -> Result<hecs::Entity> {
							spawn_door(pos, rot, open_on_exit, key, physics, world, state)
						},
					));
				}
				else if object.name.starts_with("Robot")
				{
					spawn_fns.push(Box::new(
						move |physics, world, state| -> Result<hecs::Entity> {
							spawn_robot(pos, rot, physics, world, state)
						},
					));
				}
				else if object.name.starts_with("Item")
				{
					let kind_str = object
						.properties
						.as_object()
						.and_then(|o| o.get("kind"))
						.and_then(|s| s.as_str())
						.unwrap_or("");

					let kind = comps::ItemKind::from_str(&kind_str).ok_or_else(|| {
						format!(
							"Unknown item type '{}' for Item object: {}",
							kind_str, object.name
						)
					})?;
					if kind == comps::ItemKind::Gift
					{
						num_gifts += 1;
					}
					spawn_fns.push(Box::new(
						move |physics, world, state| -> Result<hecs::Entity> {
							spawn_item(pos, Vector3::zeros(), kind, physics, world, state)
						},
					));
				}
				else if object.name.starts_with("Reactor")
				{
					spawn_fns.push(Box::new(
						move |physics, world, state| -> Result<hecs::Entity> {
							spawn_reactor(pos, rot, physics, world, state)
						},
					));
				}
				else if object.name.starts_with("ExitTrigger")
				{
					spawn_fns.push(Box::new(
						move |physics, world, _state| -> Result<hecs::Entity> {
							spawn_exit_trigger(pos, rot, physics, world)
						},
					));
				}
			}
			_ => (),
		}
	}

	let player = if let Some(player_spawn_fn) = player_spawn_fn
	{
		player_spawn_fn(physics, world, state)?
	}
	else
	{
		return Err(format!("No PlayerStart in {}", scene_name))?;
	};

	for spawn_fn in spawn_fns
	{
		spawn_fn(physics, world, state)?;
	}

	Ok(LevelProperties {
		level: entity,
		level_map: level_map,
		player: player,
		num_gifts: num_gifts,
	})
}

fn set_connector_position(
	connector: &comps::Connector, position: &mut comps::Position, start_position: &comps::Position,
	end_position: &comps::Position, state: &game_state::GameState,
)
{
	let start_pos = start_position.pos + start_position.rot * connector.start_offset;
	let end_pos = end_position.pos + end_position.rot * connector.end_offset;

	let dir = (end_pos - start_pos).normalize();
	let rot =
		UnitQuaternion::from_axis_angle(&Unit::new_unchecked(dir), 5. * state.hs.time() as f32);
	let new_pos = ((start_pos.coords + end_pos.coords) / 2.0).into();
	let new_rot = rot * UnitQuaternion::face_towards(&dir, &Vector3::y());
	let new_scale = Vector3::new(1., 1., (start_pos - end_pos).norm() / 2.);

	position.pos = new_pos;
	position.rot = new_rot;
	position.scale = new_scale;
	position.snapshot();
}

fn make_rectangle(display: &mut Display, prim: &PrimitivesAddon) -> VertexBuffer<Vertex>
{
	let vtx = Vertex {
		x: 0.0,
		y: 0.0,
		z: 0.0,
		color: Color::from_rgb_f(1., 1., 1.),
		u: 0.,
		v: 0.,
	};
	let rect_vertex_buffer = VertexBuffer::new(
		display,
		&prim,
		Some(&[
			Vertex {
				x: 0.0,
				y: 0.0,
				..vtx
			},
			Vertex {
				x: 1.0,
				y: 0.0,
				..vtx
			},
			Vertex {
				x: 1.0,
				y: 1.0,
				..vtx
			},
			Vertex {
				x: 0.0,
				y: 1.0,
				..vtx
			},
		]),
		4,
		BUFFER_STATIC,
	)
	.unwrap();
	rect_vertex_buffer
}

#[derive(Serialize, Deserialize, Clone)]
struct LevelDesc
{
	scene: String,
	name: String,
}

struct Map
{
	world: hecs::World,
	physics: Physics,
	camera_target: comps::Position,
	camera: hecs::Entity,
	player: hecs::Entity,
	accept_input: bool,
	level: hecs::Entity,
	delayed_effects: Vec<(comps::Effect, hecs::Entity, Option<hecs::Entity>)>,
	show_map: bool,
	map_rot: UnitQuaternion<f32>,
	map_zoom: f32,
	level_map: LevelMap,
	num_gifts: i32,
	damage_rectangle: VertexBuffer<Vertex>,
	level_desc: LevelDesc,
}

impl Map
{
	fn new(level_desc: &str, state: &mut game_state::GameState) -> Result<Self>
	{
		let level_desc: LevelDesc = utils::load_config(level_desc)?;

		let mut world = hecs::World::new();

		game_state::cache_scene(state, "data/sphere.glb")?;
		game_state::cache_scene(state, "data/test.obj")?;
		state.cache_bitmap("data/map_texture.png")?;
		state.cache_bitmap("data/damage_arrow.png")?;
		state.cache_bitmap("data/hud.png")?;
		state.cache_bitmap("data/hud_key.png")?;
		state.cache_bitmap("data/hud_gift.png")?;

		let mut physics = Physics::new();
		let LevelProperties {
			level,
			level_map,
			player,
			num_gifts,
		} = spawn_level(&level_desc.scene, &mut physics, &mut world, state)?;

		let player_position = { (*world.get::<&comps::Position>(player)?).clone() };
		let map_rot =
			UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -PI / 2.) * player_position.rot;

		Ok(Self {
			world: world,
			physics: physics,
			camera_target: player_position,
			player: player,
			accept_input: true,
			camera: player,
			level: level,
			delayed_effects: vec![],
			show_map: false,
			map_rot: map_rot,
			map_zoom: 40.,
			level_map: level_map,
			num_gifts: num_gifts,
			damage_rectangle: make_rectangle(state.hs.display.as_mut().unwrap(), &state.hs.prim),
			level_desc: level_desc,
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
		let in_out = state.controls.get_action_state(game_state::Action::ZoomIn)
			- state.controls.get_action_state(game_state::Action::ZoomOut);

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
		let want_enrage = state.controls.get_action_state(game_state::Action::Enrage) > 0.5;
		state
			.controls
			.clear_action_state(game_state::Action::Enrage);

		let mut player_position = None;
		if self.world.contains(self.player)
		{
			player_position = self
				.world
				.get::<&comps::Position>(self.player)
				.ok()
				.map(|v| (*v).clone());
			let mut controller = self
				.world
				.get::<&mut comps::Controller>(self.player)
				.unwrap();
			if self.show_map
			{
				let y_rot =
					UnitQuaternion::from_axis_angle(&Vector3::y_axis(), DT * rot_right_left);
				let x_rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), DT * rot_up_down);

				self.map_rot = y_rot * self.map_rot * x_rot;
				self.map_zoom *= (0.9_f32).powf(in_out);

				controller.want_move = Vector3::zeros();
				controller.want_rotate = Vector3::zeros();
			}
			else if self.accept_input
			{
				controller.want_move = Vector3::new(right_left, up_down, 0.);
				controller.want_rotate = Vector3::new(-rot_up_down, -rot_right_left, 0.);
				controller.want_enrage = want_enrage;

				for (idx, action) in [game_state::Action::GripLeft, game_state::Action::GripRight]
					.iter()
					.enumerate()
				{
					controller.want_gripper[idx] = state.controls.get_action_state(*action) > 0.5;
					state.controls.clear_action_state(*action);
				}
			}

			if !self.show_map
			{
				// LevelMap visibility.
				// Depth pass.
				let project = self.make_project_raw(
					self.level_map.depth_output.get_width() as f32,
					self.level_map.depth_output.get_height() as f32,
				);
				let camera = self.make_camera(0.);

				state
					.hs
					.core
					.set_target_bitmap(Some(&self.level_map.depth_output));
				state
					.hs
					.core
					.use_projection_transform(&utils::mat4_to_transform(project.to_homogeneous()));
				state
					.hs
					.core
					.use_transform(&utils::mat4_to_transform(camera.to_homogeneous()));
				state
					.hs
					.core
					.use_shader(Some(state.map_depth_shader.as_ref().unwrap()))
					.unwrap();

				unsafe {
					gl::Enable(gl::CULL_FACE);
					gl::CullFace(gl::BACK);
					gl::DepthMask(gl::TRUE);
				}
				state.hs.core.clear_depth_buffer(1.);
				state.hs.core.set_depth_test(Some(DepthFunction::Less));
				state
					.hs
					.core
					.set_blender(BlendOperation::Add, BlendMode::One, BlendMode::Zero);

				state.hs.core.set_shader_uniform("visible", &[0.][..]).ok();
				for (_, (position, scene)) in self
					.world
					.query::<(&comps::Position, &comps::MapScene)>()
					.iter()
				{
					if !scene.occluding
					{
						continue;
					}
					let shift = Isometry3 {
						translation: position.pos.coords.into(),
						rotation: position.rot,
					}
					.to_homogeneous();
					let scale = Matrix4::new_nonuniform_scaling(&position.scale);

					let pos_fn = |obj_pos: Point3<f32>,
					              obj_rot: UnitQuaternion<f32>,
					              obj_scale: Vector3<f32>| {
						let obj_shift = Isometry3 {
							translation: obj_pos.coords.into(),
							rotation: obj_rot.into(),
						}
						.to_homogeneous();
						let obj_scale = Matrix4::new_nonuniform_scaling(&obj_scale);

						state.hs.core.use_transform(&utils::mat4_to_transform(
							camera.to_homogeneous() * shift * scale * obj_shift * obj_scale,
						));
					};

					state.get_scene(&scene.scene).unwrap().draw(
						&state.hs.core,
						&state.hs.prim,
						|_, _| None,
						|_, _| None,
						pos_fn,
					);
				}

				let pos_fn = |obj_pos: Point3<f32>,
				              obj_rot: UnitQuaternion<f32>,
				              obj_scale: Vector3<f32>| {
					let obj_shift = Isometry3 {
						translation: obj_pos.coords.into(),
						rotation: obj_rot.into(),
					}
					.to_homogeneous();
					let obj_scale = Matrix4::new_nonuniform_scaling(&obj_scale);

					state.hs.core.use_transform(&utils::mat4_to_transform(
						camera.to_homogeneous() * obj_shift * obj_scale,
					));
				};

				state.hs.core.set_shader_uniform("visible", &[1.][..]).ok();
				self.level_map.object.draw(
					&state.hs.core,
					&state.hs.prim,
					None,
					|_, _| None,
					pos_fn,
				);

				// Exploration pass.
				state
					.hs
					.core
					.set_target_bitmap(Some(&self.level_map.exploration_buffer));
				state
					.hs
					.core
					.use_shader(Some(state.exploration_shader.as_ref().unwrap()))
					.unwrap();

				state
					.hs
					.core
					.use_projection_transform(&utils::mat4_to_transform(project.to_homogeneous()));
				unsafe {
					gl::Disable(gl::CULL_FACE);
					gl::DepthMask(gl::FALSE);
				}
				state.hs.core.set_depth_test(None);
				state
					.hs
					.core
					.set_blender(BlendOperation::Add, BlendMode::One, BlendMode::One);
				self.level_map.object.draw(
					&state.hs.core,
					&state.hs.prim,
					None,
					|_, _| Some(&self.level_map.depth_output),
					pos_fn,
				);
				self.level_map.dirty = true;
			}
		}
		if let Ok(position) = self.world.get::<&comps::Position>(self.camera)
		{
			self.camera_target.pos = position.pos;
			self.camera_target.rot = position.rot;
			self.camera_target.scale = position.scale;
		}
		if state.controls.get_action_state(game_state::Action::Pause) > 0.5
		{
			state.controls.clear_action_state(game_state::Action::Pause);
			state.hs.paused = !state.hs.paused;
		}
		if state.controls.get_action_state(game_state::Action::Map) > 0.5 && self.accept_input
		{
			state.controls.clear_action_state(game_state::Action::Map);
			self.show_map = !self.show_map;
		}
		if self.show_map && self.level_map.dirty
		{
			let w = self.level_map.exploration_buffer.get_width();
			let h = self.level_map.exploration_buffer.get_height();
			let exploration_buffer_allegro = self.level_map.exploration_buffer.get_allegro_bitmap();
			let _lock =
				self.level_map
					.exploration_buffer
					.lock(0, 0, w, h, PixelFormat::Argb8888, false);

			if let slhack::scene::ObjectKind::MultiMesh { meshes } = &mut self.level_map.object.kind
			{
				for mesh in meshes
				{
					for vtx in &mut mesh.vtxs
					{
						let r = 2;
						let mut visible = false;
						for dy in -r..=r
						{
							for dx in -r..=r
							{
								let x = (vtx.u2 * w as f32 + 0.5 as f32) as i32 + dx;
								let y = (vtx.v2 * h as f32 + 0.5 as f32) as i32 + dy;
								let color =
									unsafe { al_get_pixel(exploration_buffer_allegro, x, h - y) };
								visible |= color.r > 0.;
							}
						}
						if visible
						{
							vtx.color = Color::from_rgb_f(1., 1., 1.);
						}
						else
						{
							vtx.color = Color::from_rgba_f(0., 0., 0., 0.);
						}
					}
					let len = mesh.vertex_buffer.len();
					let mut lock = mesh.vertex_buffer.lock_write_only(0, len).unwrap();
					lock.copy_from_slice(&mesh.vtxs);
				}
			}
			self.level_map.dirty = false;
		}

		// AI.
		for (_, (position, controller, physics, ai)) in self
			.world
			.query::<(
				&comps::Position,
				&mut comps::Controller,
				&comps::Physics,
				&mut comps::AI,
			)>()
			.iter()
		{
			let mut new_state = None;
			match ai.state
			{
				comps::AIState::Idle =>
				{
					if let Some(player_position) = player_position
					{
						let dir = (player_position.pos - position.pos).normalize();
						let (forward, _, _) = get_dirs(position.rot);
						if forward.dot(&dir) > 0.
						{
							if let Some((collider_handle, range)) =
								self.physics.ray_cast(position.pos, dir, physics.handle)
							{
								let collider =
									self.physics.collider_set.get(collider_handle).unwrap();
								if hecs::Entity::from_bits(collider.user_data as u64).unwrap()
									== self.player && range < 5.
								{
									new_state = Some(comps::AIState::Attacking(self.player))
								}
							}
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
							if diff.norm() > 5.
							{
								controller.want_move.z = 1.;
							}
							else if diff.norm() < 3.
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
		for (_, (position, explosion_scaling)) in self
			.world
			.query::<(&mut comps::Position, &comps::ExplosionScaling)>()
			.iter()
		{
			position.scale += Vector3::from_element(explosion_scaling.scale_rate * DT);
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
		for (_, (controller, stats, physics)) in self
			.world
			.query::<(&mut comps::Controller, &comps::Stats, &comps::Physics)>()
			.iter()
		{
			let body = self.physics.rigid_body_set.get_mut(physics.handle).unwrap();

			let rot = body.rotation().clone();
			let (forward, right, up) = get_dirs(rot);

			body.add_force(
				stats.cur.speed
					* (controller.want_move.x * right
						+ controller.want_move.y * up
						+ controller.want_move.z * forward),
				true,
			);
			//body.apply_torque_impulse(DT * (rot * controller.want_rotate), true);
			body.set_angvel(
				DT * stats.cur.rot_speed * (rot * controller.want_rotate) + body.angvel(),
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

					if let Ok(on_collide_effects) = self.world.get::<&comps::OnCollideEffects>(id)
					{
						for effect in &on_collide_effects.effects
						{
							effects.push((effect.clone(), id, Some(other_id)));
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
						let mut parent_query = self
							.world
							.query_one::<(&comps::Grippers, &mut comps::Physics)>(gripper.parent)
							.ok();
						if let Some((grippers, parent_physics)) =
							parent_query.as_mut().and_then(|q| q.get())
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

								let joint =
									SpringJointBuilder::new(0.1, 30. * grippers.power_level, 10.)
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
								if let Some((_, target_physics)) = self
									.world
									.query_one::<(&comps::Health, &comps::Physics)>(other_id)
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
										Some(other_id),
									));
									effects.push((comps::Effect::SpawnHit, id, Some(other_id)));
									self.delayed_effects.push((
										comps::Effect::GripperPierce {
											old_vel: 0.75 * gripper_vel,
										},
										id,
										Some(other_id),
									));
								}
								else
								{
									if self.world.get::<&comps::Door>(other_id).is_ok()
									{
										effects.push((comps::Effect::SpawnHit, id, Some(other_id)));
									}
									// This is an effect because we want to spawn the hit at a
									// location before we attach it.
									self.delayed_effects.push((
										comps::Effect::ReattachGripper,
										id,
										None,
									));
								}
							}
						}
					}
				}
			}
		}

		// Gripper death.
		for (id, gripper) in self.world.query::<&comps::Gripper>().iter()
		{
			if !self.world.contains(gripper.parent)
			{
				to_die.push(id);
			}
		}

		// Stats.
		for (_, (stats, grippers)) in self
			.world
			.query::<(&mut comps::Stats, &comps::Grippers)>()
			.iter()
		{
			stats.update(grippers.power_level);
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
			if let Ok(mut health) = self.world.get::<&mut comps::Health>(id)
			{
				if controller.want_enrage
				{
					health.health -= 10.;
					health.damage_time = state.hs.time;
					grippers.last_kill_time = state.hs.time;
					grippers.power_level += 0.1;
				}
			}
			for (want_gripper, gripper_id) in
				itertools::izip!(&controller.want_gripper, &mut grippers.grippers,)
			{
				if *want_gripper || controller.force_attach
				{
					want_grip.push((
						id,
						*position,
						*gripper_id,
						grippers.power_level,
						controller.force_attach,
					));
				}
			}
			if state.hs.time > grippers.last_kill_time + POWER_LEVEL_TIME
			{
				grippers.power_level = 1.0;
			}
		}

		for (id, parent_position, gripper_id, power_level, force_attach) in want_grip
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
						if state.hs.time() >= gripper.time_to_grip && !force_attach
						{
							let gripper_body = self
								.physics
								.rigid_body_set
								.get_mut(gripper_physics.handle)
								.unwrap();
							gripper_body.apply_impulse(power_level * forward, true);
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
			if physics.copy_rot
			{
				position.rot = *body.rotation();
			}
			physics.old_vel = *body.linvel();
		}

		//  FaceTowards.
		let mut new_rot = vec![];
		for (id, (position, face_towards)) in self
			.world
			.query::<(&comps::Position, &mut comps::FaceTowards)>()
			.iter()
		{
			if let Ok(target_position) = self.world.get::<&comps::Position>(face_towards.target)
			{
				face_towards.last_pos = target_position.pos;
			}
			new_rot.push((
				id,
				UnitQuaternion::face_towards(
					&-(face_towards.last_pos - position.pos),
					&Vector3::y_axis(),
				),
			));
		}
		for (id, new_rot) in new_rot
		{
			self.world.get::<&mut comps::Position>(id).unwrap().rot = new_rot;
		}

		// Door upkeep.
		for (id, (position, door, map_scene, scene, physics)) in self
			.world
			.query::<(
				&comps::Position,
				&mut comps::Door,
				&mut comps::MapScene,
				&mut comps::Scene,
				&comps::Physics,
			)>()
			.iter()
		{
			if let Some(player_position) = player_position
				&& !map_scene.explored
			{
				if let Ok(player_physics) = self.world.get::<&comps::Physics>(self.player)
				{
					let dir = position.pos - player_position.pos;
					let (forward, _, _) = get_dirs(player_position.rot);
					if forward.dot(&dir) > 0.
					{
						if let Some((collider_handle, _)) =
							self.physics
								.ray_cast(player_position.pos, dir, player_physics.handle)
						{
							let collider = self.physics.collider_set.get(collider_handle).unwrap();
							if hecs::Entity::from_bits(collider.user_data as u64).unwrap() == id
							{
								map_scene.explored = true;
							}
						}
					}
				}
			}

			map_scene.occluding = door.status == comps::DoorStatus::Closed;

			if door.status == comps::DoorStatus::Open
				&& state.hs.time > door.time_to_close
				&& !door.open_on_exit
			{
				let real_scene = state.get_scene(&scene.scene).unwrap();

				let mut animation_states = HashMap::new();
				for (idx, obj) in real_scene.objects.iter().enumerate()
				{
					if obj.name.starts_with("Door")
					{
						let animation_state = comps::AnimationState {
							speed: 1.0,
							state: scene::AnimationState::new("Close", true),
						};
						animation_states.insert(idx as i32, animation_state);
					}
				}
				scene.animation_states = animation_states;

				let body = self.physics.rigid_body_set.get(physics.handle).unwrap();
				for collider_handle in body.colliders()
				{
					let collider = self.physics.collider_set.get_mut(*collider_handle).unwrap();
					collider.set_collision_groups(InteractionGroups::all());
				}
				door.status = comps::DoorStatus::Closed;
			}
			else if door.want_open
			{
				effects.push((comps::Effect::Open, id, None));
			}
		}

		// Scene upkeep.
		for (_, scene) in self.world.query::<&mut comps::Scene>().iter()
		{
			let the_scene = state.get_scene(&scene.scene)?;
			for (obj_idx, animation_state) in &mut scene.animation_states
			{
				the_scene.objects[*obj_idx as usize].advance_state(
					&mut animation_state.state,
					(animation_state.speed * DT) as f64,
				);
			}
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
			set_connector_position(&connector, position, &start_position, &end_position, state);
		}

		// SceneObjectPosition.
		for (_, (scene_object_position, position)) in self
			.world
			.query::<(&mut comps::SceneObjectPosition, &mut comps::Position)>()
			.iter()
		{
			let scene = state.get_scene(&scene_object_position.scene_name)?;
			let object = &scene.objects[scene_object_position.object_idx as usize];

			object.advance_state(&mut scene_object_position.animation_state, DT as f64);
			let (pos, rot, scale) =
				object.get_animation_position(&scene_object_position.animation_state);

			//let fixed_rot = Unit::from_quaternion(Quaternion::new(rot.i, -rot.k, rot.j, rot.w));
			let fixed_rot = rot * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -PI / 2.);

			position.pos = pos;
			position.rot = fixed_rot;
			position.scale = scale;
		}

		// PositionCopier
		for (_, (position_copier, position)) in self
			.world
			.query::<(&comps::PositionCopier, &comps::Position)>()
			.iter()
		{
			if let Ok(physics) = self.world.get::<&comps::Physics>(position_copier.target)
			{
				let body = self.physics.rigid_body_set.get_mut(physics.handle).unwrap();
				body.set_position(position.pos.into(), true);
				body.set_rotation(position.rot, true);
			}
		}

		// Explosion spawner
		for (_, (explosion_spawner, position)) in self
			.world
			.query::<(&mut comps::ExplosionSpawner, &comps::Position)>()
			.iter()
		{
			if state.hs.time > explosion_spawner.time_for_explosion
			{
				let mut rng = rand::rng();
				let pos = position.pos
					+ Vector3::new(
						rng.random_range(-1.0..=1.0),
						rng.random_range(-1.0..=1.0),
						rng.random_range(-1.0..=1.0),
					);
				let kind = explosion_spawner.kind;
				spawn_fns.push(Box::new(move |map, state| -> Result<hecs::Entity> {
					spawn_explosion(pos, kind, &mut map.world, state)
				}));
				explosion_spawner.time_for_explosion = state.hs.time + 0.1;
			}
		}

		// Spawn fns;
		for spawn_fn in spawn_fns
		{
			spawn_fn(self, state)?;
		}

		// Health.
		for (id, health) in self.world.query::<&mut comps::Health>().iter()
		{
			if health.recovery > 0. && health.health < health.max_health
			{
				let amt = (health.max_health - health.health)
					.min(DT * 5.)
					.min(health.recovery);
				health.health += amt;
				health.recovery -= amt;
			}
			if health.health <= 0.0 && !health.dead
			{
				health.dead = true;
				for effect in &health.death_effects
				{
					effects.push((effect.clone(), id, None));
				}
				if health.remove_on_death
				{
					to_die.push(id);
					if let Ok(on_death_effects) = self.world.get::<&comps::OnDeathEffects>(id)
					{
						for effect in &on_death_effects.effects
						{
							effects.push((effect.clone(), id, None));
						}
					}
				}
				if let Some(mut grippers) = health
					.damaged_by
					.and_then(|other_id| self.world.get::<&mut comps::Grippers>(other_id).ok())
				{
					grippers.last_kill_time = state.hs.time();
					grippers.power_level += 0.5;
				}
			}
		}

		// TimeToDie.
		for (id, time_to_die) in self.world.query::<&mut comps::TimeToDie>().iter()
		{
			if state.hs.time > time_to_die.time_to_die && !time_to_die.dead
			{
				time_to_die.dead = true;
				to_die.push(id);
				if let Ok(on_death_effects) = self.world.get::<&comps::OnDeathEffects>(id)
				{
					for effect in &on_death_effects.effects
					{
						effects.push((effect.clone(), id, None));
					}
				}
			}
		}

		// Effects.
		while !effects.is_empty()
		{
			let mut new_effects = vec![];
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
						if let Ok(mut health) =
							self.world.get::<&mut comps::Health>(other_id.unwrap())
						{
							health.health -= amount;
							health.damage_time = state.hs.time;
							health.damaged_by = Some(owner);
						}
						if let Ok(mut ai) = self.world.get::<&mut comps::AI>(other_id.unwrap())
						{
							ai.state = comps::AIState::Attacking(owner);
						}
					}
					comps::Effect::GripperPierce { old_vel } =>
					{
						if self.world.contains(other_id.unwrap())
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
						if let Ok(position) = self.world.get::<&comps::Position>(other_id.unwrap())
						{
							dest_pos = Some(position.pos);
						}
						if let (Some(src_pos), Some(dest_pos)) = (src_pos, dest_pos)
						{
							let dir = (dest_pos - src_pos).normalize();
							let rot = UnitQuaternion::face_towards(&dir, &Vector3::y());
							spawn_hit(src_pos + 0.3 * dir, rot, &mut self.world, state)?;
						}
					}
					comps::Effect::SpawnExplosion { kind } =>
					{
						let mut src_pos = None;
						if let Ok(position) = self.world.get::<&comps::Position>(id)
						{
							src_pos = Some(position.pos);
						}
						if let Some(src_pos) = src_pos
						{
							spawn_explosion(src_pos, kind, &mut self.world, state)?;
						}
					}
					comps::Effect::Open =>
					{
						let mut query = self
							.world
							.query_one::<(&mut comps::Door, &mut comps::Scene, &comps::Physics)>(id)
							.unwrap();
						if let Some((door, scene, physics)) = query.get()
						{
							if door.open_on_exit && other_id.is_some()
							{
								// Don't open on touch.
								continue;
							}
							if let Some(key) = door.key
							{
								let mut do_open = false;
								if let Some(mut other_id) = other_id
								{
									if let Ok(gripper) = self.world.get::<&comps::Gripper>(other_id)
									{
										other_id = gripper.parent;
									}
									if let Ok(inventory) =
										self.world.get::<&comps::Inventory>(other_id)
									{
										do_open = inventory.keys.contains(&key);
									}
								}
								if !do_open
								{
									continue;
								}
							}

							let mut can_open = true;
							for animation_state in scene.animation_states.values()
							{
								can_open = animation_state.state.get_num_loops() != 0;
							}

							if can_open
							{
								let real_scene = state.get_scene(&scene.scene).unwrap();

								let mut animation_states = HashMap::new();
								for (idx, obj) in real_scene.objects.iter().enumerate()
								{
									if obj.name.starts_with("Door")
									{
										let animation_state = comps::AnimationState {
											speed: 1.0,
											state: scene::AnimationState::new("Open", true),
										};
										animation_states.insert(idx as i32, animation_state);
									}
								}
								scene.animation_states = animation_states;

								let body = self.physics.rigid_body_set.get(physics.handle).unwrap();
								for collider_handle in body.colliders()
								{
									let collider = self
										.physics
										.collider_set
										.get_mut(*collider_handle)
										.unwrap();
									collider.set_collision_groups(InteractionGroups::none());
								}
								door.time_to_close = state.hs.time + 5.;
								door.status = comps::DoorStatus::Open;
								door.want_open = false;
							}
							else
							{
								door.want_open = true;
							}
						}
					}
					comps::Effect::StartExitAnimation =>
					{
						spawn_exit_explosions(
							&self.level_desc.scene,
							"ExitExplosions",
							&mut self.world,
							state,
						)?;
						spawn_player_exit_track(
							&self.level_desc.scene,
							"ExitTrack",
							self.player,
							&mut self.world,
							state,
						)?;
						self.camera = spawn_animated_target(
							&self.level_desc.scene,
							"ExitCamera",
							&mut self.world,
							state,
						)?;
						if let Ok(mut controller) =
							self.world.get::<&mut comps::Controller>(self.player)
						{
							controller.force_attach = true;
						}
						self.accept_input = false;
					}
					comps::Effect::OpenExit =>
					{
						for (id, door) in self.world.query::<&comps::Door>().iter()
						{
							if door.open_on_exit
							{
								new_effects.push((comps::Effect::Open, id, None));
							}
						}
					}
					comps::Effect::ExplosionSpawner { kind } =>
					{
						self.world
							.insert_one(id, comps::ExplosionSpawner::new(kind))?;
					}
					comps::Effect::SpawnDeathCamera =>
					{
						self.accept_input = false;
						self.show_map = false;
						let mut src_pos = None;
						if let Ok(position) = self.world.get::<&comps::Position>(id)
						{
							src_pos = Some(position.pos.clone());
						}
						if let Some(src_pos) = src_pos
						{
							self.camera = spawn_death_camera(
								src_pos,
								id,
								&mut self.physics,
								&mut self.world,
							)?;
						}
					}
					comps::Effect::DelayedDeath { delay } =>
					{
						self.world
							.insert_one(id, comps::TimeToDie::new(state.hs.time + delay))?;
					}
					comps::Effect::PickupItem { kind } =>
					{
						if let Some(other_id) = other_id
						{
							if other_id == self.player
							{
								match kind
								{
									comps::ItemKind::Energy =>
									{
										if let Ok(mut health) =
											self.world.get::<&mut comps::Health>(self.player)
										{
											health.recovery = (health.recovery + ENERGY_AMOUNT)
												.min(health.max_recovery);
										}
									}
									comps::ItemKind::Key { kind } =>
									{
										if let Ok(mut inventory) =
											self.world.get::<&mut comps::Inventory>(self.player)
										{
											inventory.keys.insert(kind);
										}
									}
									comps::ItemKind::Gift =>
									{
										if let Ok(mut inventory) =
											self.world.get::<&mut comps::Inventory>(self.player)
										{
											inventory.num_gifts += 1;
										}
									}
								}
								to_die.push(id);
							}
						}
					}
					comps::Effect::SpawnItem { spawn_table } =>
					{
						let mut src_pos = None;
						if let Ok(position) = self.world.get::<&comps::Position>(id)
						{
							src_pos = Some(position.pos);
						}
						if let Some(src_pos) = src_pos
						{
							let total_prob: f32 = spawn_table.iter().map(|(w, _)| w).sum();
							let mut rng = rand::rng();
							if rng.random_range(0.0..1.0) < total_prob
							{
								let item_kind = spawn_table
									.choose_weighted(&mut rng, |&(w, _)| w)
									.unwrap()
									.1;
								spawn_item(
									src_pos,
									0.5 * Vector3::<f64>::from_row_slice(
										&rand_distr::UnitSphere.sample(&mut rng),
									)
									.cast::<f32>(),
									item_kind,
									&mut self.physics,
									&mut self.world,
									state,
								)?;
							}
						}
					}
					comps::Effect::ReattachGripper =>
					{
						let mut do_attach = false;
						if let Ok(mut gripper) = self.world.get::<&mut comps::Gripper>(id)
						{
							gripper.status = comps::GripperStatus::AttachedToParent;
							if let Some(connector_id) = gripper.connector.take()
							{
								to_die.push(connector_id);
							}
							do_attach = true;
						}
						if do_attach
						{
							attach_gripper_to_parent(id, &self.world, &mut self.physics);
						}
					}
				}
			}

			effects = new_effects;
		}

		// Remove dead entities
		to_die.sort();
		to_die.dedup();
		for id in to_die.drain(..)
		{
			//println!("Dead: {:?}", id);
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

	fn make_project_raw(&self, buffer_width: f32, buffer_height: f32) -> Perspective3<f32>
	{
		Perspective3::new(buffer_width / buffer_height, PI / 3., 0.1, 100.)
	}

	fn make_project(&self, state: &game_state::GameState) -> Perspective3<f32>
	{
		let dw = state.hs.buffer_width();
		let dh = state.hs.buffer_height();
		self.make_project_raw(dw, dh)
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

	fn make_map_camera(&self, alpha: f32) -> Isometry3<f32>
	{
		let (forward, _, _) = get_dirs(self.map_rot);
		Isometry3 {
			rotation: self.map_rot,
			translation: (self.camera_target.draw_pos(alpha).coords - self.map_zoom * forward)
				.into(),
		}
		.inverse()
	}

	fn draw(&mut self, state: &mut game_state::GameState) -> Result<()>
	{
		let alpha = state.hs.alpha;
		let project = self.make_project(state);

		let camera = if self.show_map
		{
			self.make_map_camera(alpha)
		}
		else
		{
			self.make_camera(alpha)
		};

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

		let material_mapper = |material: &scene::Material<game_state::MaterialKind>,
		                       texture_name: &str|
		 -> Option<&Bitmap> {
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
						state
							.get_bitmap(&material.desc.lightmap)
							.into_slhack()
							.unwrap(),
						1,
					)
					.ok();
			}
			state.get_bitmap(texture_name).ok()
		};

		if self.show_map
		{
			let pos_fn =
				|obj_pos: Point3<f32>, obj_rot: UnitQuaternion<f32>, obj_scale: Vector3<f32>| {
					let obj_shift = Isometry3 {
						translation: obj_pos.coords.into(),
						rotation: obj_rot.into(),
					}
					.to_homogeneous();
					let obj_scale = Matrix4::new_nonuniform_scaling(&obj_scale);

					state.hs.core.use_transform(&utils::mat4_to_transform(
						camera.to_homogeneous() * obj_shift * obj_scale,
					));
					state
						.hs
						.core
						.set_shader_transform(
							"model_matrix",
							&utils::mat4_to_transform(obj_shift * obj_scale),
						)
						.ok();
				};
			state
				.hs
				.core
				.set_shader_uniform("base_color", &[[1.; 4]][..])
				.ok();
			state
				.hs
				.core
				.set_shader_uniform("time", &[state.hs.time as f32][..])
				.ok();
			let player_pos = [
				self.camera_target.pos.x,
				self.camera_target.pos.y,
				self.camera_target.pos.z,
			];
			state
				.hs
				.core
				.set_shader_uniform("player_pos", &[player_pos][..])
				.ok();

			self.level_map.object.draw(
				&state.hs.core,
				&state.hs.prim,
				None,
				material_mapper,
				pos_fn,
			);

			let mut query = self.world.query::<(&comps::Position, &comps::MapScene)>();
			for (_, (position, scene)) in query.iter()
			{
				if !scene.explored
				{
					continue;
				}
				let shift = Isometry3 {
					translation: position.draw_pos(alpha).coords.into(),
					rotation: position.draw_rot(alpha),
				}
				.to_homogeneous();
				let scale = Matrix4::new_nonuniform_scaling(&position.draw_scale(alpha));

				let pos_fn = |obj_pos: Point3<f32>,
				              obj_rot: UnitQuaternion<f32>,
				              obj_scale: Vector3<f32>| {
					let obj_shift = Isometry3 {
						translation: obj_pos.coords.into(),
						rotation: obj_rot.into(),
					}
					.to_homogeneous();
					let obj_scale = Matrix4::new_nonuniform_scaling(&obj_scale);

					state.hs.core.use_transform(&utils::mat4_to_transform(
						camera.to_homogeneous() * shift * scale * obj_shift * obj_scale,
					));
					state
						.hs
						.core
						.set_shader_transform(
							"model_matrix",
							&utils::mat4_to_transform(shift * scale * obj_shift * obj_scale),
						)
						.ok();
				};
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
					|idx, _| scene.animation_states.get(&(idx as i32)).map(|s| &s.state),
					material_mapper,
					pos_fn,
				);
			}
		}
		else
		{
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

				let pos_fn = |obj_pos: Point3<f32>,
				              obj_rot: UnitQuaternion<f32>,
				              obj_scale: Vector3<f32>| {
					let obj_shift = Isometry3 {
						translation: obj_pos.coords.into(),
						rotation: obj_rot.into(),
					}
					.to_homogeneous();
					let obj_scale = Matrix4::new_nonuniform_scaling(&obj_scale);

					state.hs.core.use_transform(&utils::mat4_to_transform(
						camera.to_homogeneous() * shift * scale * obj_shift * obj_scale,
					));
					state
						.hs
						.core
						.set_shader_transform(
							"model_matrix",
							&utils::mat4_to_transform(shift * scale * obj_shift * obj_scale),
						)
						.ok();
				};
				// THIS IS HORRIBLE
				let color = scene.color;
				let (r, g, b, a) = color.to_rgba_f();
				let color = [r, g, b, a];
				state
					.hs
					.core
					.set_shader_uniform("base_color", &[color][..])
					.ok();
				state
					.hs
					.core
					.set_shader_uniform("time", &[state.hs.time as f32][..])
					.ok();

				state.get_scene(&scene.scene).unwrap().draw(
					&state.hs.core,
					&state.hs.prim,
					|idx, _| scene.animation_states.get(&(idx as i32)).map(|s| &s.state),
					material_mapper,
					pos_fn,
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

		if !self.show_map
		{
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
					scene.draw(
						&state.hs.core,
						&state.hs.prim,
						|_, _| None,
						|_, s| state.get_bitmap(s).ok(),
						|_, _, _| {},
					);
				}
			}
		}

		// Final pass.
		unsafe {
			gl::DepthMask(gl::FALSE);
		}
		state.deferred_renderer.as_mut().unwrap().final_pass(
			&state.hs.core,
			&state.hs.prim,
			state.final_shader.as_ref().unwrap(),
			state.hs.buffer1.as_ref().unwrap(),
		)?;
		unsafe {
			gl::CullFace(gl::BACK);
			gl::DepthMask(gl::TRUE);
		}

		// TODO: NO WAY!
		let material_mapper = |material: &scene::Material<game_state::MaterialKind>,
		                       texture_name: &str|
		 -> Option<&Bitmap> {
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
			state.get_bitmap(texture_name).ok()
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
		if !self.show_map
		{
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

				let pos_fn = |obj_pos: Point3<f32>,
				              obj_rot: UnitQuaternion<f32>,
				              obj_scale: Vector3<f32>| {
					let obj_shift = Isometry3 {
						translation: obj_pos.coords.into(),
						rotation: obj_rot.into(),
					}
					.to_homogeneous();
					let obj_scale = Matrix4::new_nonuniform_scaling(&obj_scale);

					state.hs.core.use_transform(&utils::mat4_to_transform(
						camera.to_homogeneous() * shift * scale * obj_shift * obj_scale,
					));
				};

				state
					.hs
					.core
					.set_shader_uniform("base_color", &[color_to_array(scene.color)][..])
					.ok();

				state.get_scene(&scene.scene).unwrap().draw(
					&state.hs.core,
					&state.hs.prim,
					|idx, _| scene.animation_states.get(&(idx as i32)).map(|s| &s.state),
					material_mapper,
					pos_fn,
				);
			}
		}

		state
			.hs
			.core
			.use_shader(Some(state.basic_shader.as_ref().unwrap()))
			.unwrap();
		state
			.hs
			.core
			.set_shader_uniform("tint", &[color_to_array(Color::from_rgb_f(1., 1., 1.))][..])
			.ok();
		unsafe {
			gl::Disable(gl::CULL_FACE);
		}
		state.hs.core.set_depth_test(None);
		state
			.hs
			.core
			.set_blender(BlendOperation::Add, BlendMode::One, BlendMode::InverseAlpha);
		let ortho_mat = Matrix4::new_orthographic(
			0.,
			state.hs.buffer_width() as f32,
			state.hs.buffer_height() as f32,
			0.,
			state.hs.buffer_height(),
			-state.hs.buffer_height(),
		);
		state
			.hs
			.core
			.use_projection_transform(&utils::mat4_to_transform(ortho_mat));
		state.hs.core.use_transform(&Transform::identity());

		// HUD
		let bw = state.hs.buffer_width();
		let bh = state.hs.buffer_height();
		let cx = bw / 2.;
		let lh = state.hs.ui_font().get_line_height() as f32;

		if self.world.contains(self.player) && self.accept_input
		{
			let position = self.world.get::<&comps::Position>(self.player).unwrap();
			let health = self.world.get::<&comps::Health>(self.player).unwrap();
			let grippers = self.world.get::<&comps::Grippers>(self.player).unwrap();
			let inventory = self.world.get::<&comps::Inventory>(self.player).unwrap();
			if !health.dead
			{
				if !self.show_map
				{
					let f = health.recovery / health.max_recovery;
					let delta_theta = 2. / 3. * PI * f;
					state.hs.prim.draw_arc(
						cx - 64.,
						bh - 64.,
						22.,
						2. / 3. * PI,
						delta_theta,
						Color::from_rgb_f(0.5, 0.5, 0.8),
						8.,
					);

					let f = health.health / health.max_health;
					let delta_theta = 2. / 3. * PI * f;
					state.hs.prim.draw_arc(
						cx - 64.,
						bh - 64.,
						30.,
						2. / 3. * PI,
						delta_theta,
						Color::from_rgb_f(0.8, 0.2, 0.2),
						8.,
					);

					let percent = (f * 100.).round() as i32;
					state.hs.core.draw_text(
						state.hud_font(),
						Color::from_rgb_f(0.6, 0.8, 0.9),
						cx - 70.,
						bh - 64. - lh / 2.,
						FontAlign::Left,
						&format!("{percent}"),
					);

					let percent = (health.recovery / health.max_health * 100.).round() as i32;
					if percent > 0
					{
						state.hs.core.draw_text(
							state.small_hud_font(),
							Color::from_rgb_f(0.6, 0.8, 0.9),
							cx - 70.,
							bh - 64. - lh / 2. + 16.,
							FontAlign::Left,
							&format!("+{percent}"),
						);
					}

					let f = 1.0
						- (((state.hs.time - grippers.last_kill_time) / POWER_LEVEL_TIME) as f32)
							.min(1.);
					let delta_theta = 2. / 3. * PI * f;
					state.hs.prim.draw_arc(
						cx + 64.,
						bh - 64.,
						30.,
						1. / 3. * PI,
						-delta_theta,
						Color::from_rgb_f(0.8, 0.8, 0.2),
						8.,
					);

					let power_level = grippers.power_level;
					state.hs.core.draw_text(
						state.hud_font(),
						Color::from_rgb_f(0.8, 0.8, 0.2),
						cx + 20.,
						bh - 64. - lh / 2.,
						FontAlign::Left,
						&format!("{power_level:.1}x"),
					);
					let hud = state.get_bitmap("data/hud.png")?;
					let hud_w = hud.get_width() as f32;
					let hud_h = hud.get_height() as f32;
					state.hs.core.draw_bitmap(
						hud,
						cx - hud_w / 2.,
						bh - 64. - hud_h / 2.,
						Flag::zero(),
					);
				}

				let hud_gift = state.get_bitmap("data/hud_gift.png")?;
				let hud_gift_w = hud_gift.get_width() as f32;
				let hud_gift_h = hud_gift.get_height() as f32;

				state.hs.core.draw_tinted_bitmap(
					hud_gift,
					Color::from_rgb_f(1., 1., 1.),
					32. - hud_gift_w / 2.,
					16. + lh / 2. - hud_gift_h / 2.,
					Flag::zero(),
				);

				state.hs.core.draw_text(
					state.hud_font(),
					Color::from_rgb_f(0.6, 0.8, 0.9),
					36.,
					16.,
					FontAlign::Left,
					&format!("{:>2}/{:}", inventory.num_gifts, self.num_gifts),
				);

				let hud_key = state.get_bitmap("data/hud_key.png")?;
				let hud_key_w = hud_key.get_width() as f32;
				let hud_key_h = hud_key.get_height() as f32;

				for (i, key) in [
					comps::KeyKind::Red,
					comps::KeyKind::Yellow,
					comps::KeyKind::Blue,
				]
				.iter()
				.enumerate()
				{
					if inventory.keys.contains(key)
					{
						state.hs.core.draw_tinted_bitmap(
							hud_key,
							key.color(),
							bw - 64. - hud_key_w + 24. * i as f32,
							32. - hud_key_h,
							Flag::zero(),
						);
					}
				}

				let damage_arrow = state.get_bitmap("data/damage_arrow.png")?;
				let arrow_w = damage_arrow.get_width() as f32;
				let arrow_h = damage_arrow.get_height() as f32;
				let f = 1. - ((state.hs.time - health.damage_time) / 0.5).min(1.) as f32;

				let mut dir = None;
				if let Some(other_id) = health.damaged_by
				{
					if let Ok(other_position) = self.world.get::<&comps::Position>(other_id)
					{
						let diff = other_position.pos - position.pos;
						let (_, right, up) = get_dirs(position.rot);
						let dirs = [
							diff.dot(&right),
							-diff.dot(&up),
							-diff.dot(&right),
							diff.dot(&up),
						];
						dir = dirs
							.into_iter()
							.map(ordered_float::OrderedFloat)
							.position_max()
					}
				}

				if let Some(dir) = dir
				{
					let (dx, dy, angle) = [
						(bw - arrow_h / 2., bh / 2., PI / 2.),
						(bw / 2., bh - arrow_h / 2., PI),
						(arrow_h / 2., bh / 2., 3. * PI / 2.),
						(bw / 2., arrow_h / 2., 0.),
					][dir];

					state.hs.core.draw_tinted_rotated_bitmap(
						damage_arrow,
						Color::from_rgba_f(f * 0.5, 0., 0., f * 0.5),
						arrow_w / 2.,
						arrow_h / 2.,
						dx,
						dy,
						angle,
						Flag::zero(),
					);
				}

				state
					.hs
					.core
					.set_shader_uniform(
						"tint",
						&[color_to_array(Color::from_rgba_f(
							0.25 * f,
							0.,
							0.,
							0.25 * f,
						))][..],
					)
					.ok();
				let mut transform = Transform::identity();
				transform.translate(-0.5, -0.5);
				transform.scale(bw, bh);
				transform.translate(bw / 2., bh / 2.);
				state.hs.core.use_transform(&transform);
				state.hs.prim.draw_vertex_buffer(
					&self.damage_rectangle,
					Option::<&Bitmap>::None,
					0,
					4,
					PrimType::TriangleFan,
				);
			}
		}
		state.hs.core.use_transform(&Transform::identity());
		state
			.hs
			.core
			.set_shader_uniform("tint", &[color_to_array(Color::from_rgb_f(1., 1., 1.))][..])
			.ok();
		if self.show_map
		{
			state.hs.core.draw_text(
				state.hud_font(),
				Color::from_rgb_f(0.8, 0.8, 0.8),
				cx,
				bh - lh - 4.,
				FontAlign::Centre,
				&self.level_desc.name,
			);
		}
		// state
		// 	.hs
		// 	.core
		// 	.draw_bitmap(&self.level_map.exploration_buffer, 0., 0., Flag::zero());
		// state.hs.core.draw_bitmap(
		//  	&self.level_map.depth_output,
		//  	state.hs.buffer_width() - self.level_map.depth_output.get_width() as f32,
		//  	0.,
		//  	Flag::zero(),
		//  );
		Ok(())
	}
}
