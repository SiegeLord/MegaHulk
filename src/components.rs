use allegro::*;
use nalgebra::{Point3, Quaternion, Unit, Vector3};
use rand::prelude::*;
use rapier3d::dynamics::ImpulseJointHandle;
use slhack::sprite;
use std::collections::HashMap;

#[derive(Debug, Copy, Clone)]
pub struct Light
{
	pub color: Color,
	pub intensity: f32,
	pub static_: bool,
}

#[derive(Debug, Copy, Clone)]
pub struct Position
{
	pub pos: Point3<f32>,
	old_pos: Point3<f32>,

	pub rot: Unit<Quaternion<f32>>,
	old_rot: Unit<Quaternion<f32>>,

	pub scale: Vector3<f32>,
	old_scale: Vector3<f32>,
}

impl Position
{
	pub fn new(pos: Point3<f32>, rot: Unit<Quaternion<f32>>) -> Self
	{
		Self::new_scaled(pos, rot, Vector3::new(1., 1., 1.))
	}

	pub fn new_scaled(pos: Point3<f32>, rot: Unit<Quaternion<f32>>, scale: Vector3<f32>) -> Self
	{
		Self {
			pos: pos,
			old_pos: pos,
			rot: rot,
			old_rot: rot,
			scale: scale,
			old_scale: scale,
		}
	}

	pub fn snapshot(&mut self)
	{
		self.old_pos = self.pos;
		self.old_rot = self.rot;
		self.old_scale = self.scale;
	}

	pub fn draw_pos(&self, alpha: f32) -> Point3<f32>
	{
		self.pos + alpha * (self.pos - self.old_pos)
	}

	pub fn draw_rot(&self, alpha: f32) -> Unit<Quaternion<f32>>
	{
		self.old_rot.slerp(&self.rot, alpha)
	}

	pub fn draw_scale(&self, alpha: f32) -> Vector3<f32>
	{
		self.scale + alpha * (self.scale - self.old_scale)
	}
}

#[derive(Debug, Clone)]
pub struct AnimationState
{
	pub speed: f32,
	pub state: slhack::scene::AnimationState,
}

#[derive(Debug, Clone)]
pub struct Scene
{
	pub scene: String,
	pub color: Color,
	// This indexes into the Scene's objects
	pub animation_states: HashMap<i32, AnimationState>,
}

impl Scene
{
	pub fn new(scene: &str) -> Self
	{
		Self {
			scene: scene.to_string(),
			color: Color::from_rgb_f(1., 1., 1.),
			animation_states: HashMap::new(),
		}
	}
}

#[derive(Debug, Clone)]
pub struct MapScene
{
	pub scene: String,
	pub color: Color,
	// This indexes into the Scene's objects
	pub animation_states: HashMap<i32, AnimationState>,
	pub occluding: bool,
	pub explored: bool,
}

impl MapScene
{
	pub fn new(scene: &str) -> Self
	{
		Self {
			scene: scene.to_string(),
			color: Color::from_rgb_f(1., 1., 1.),
			animation_states: HashMap::new(),
			occluding: true,
			explored: false,
		}
	}
}

#[derive(Debug, Clone)]
pub struct AdditiveScene
{
	pub scene: String,
	pub color: Color,
	// This indexes into the Scene's objects
	pub animation_states: HashMap<i32, AnimationState>,
}

impl AdditiveScene
{
	pub fn new(scene: &str) -> Self
	{
		Self {
			scene: scene.to_string(),
			color: Color::from_rgb_f(1., 1., 1.),
			animation_states: HashMap::new(),
		}
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Physics
{
	pub handle: rapier3d::dynamics::RigidBodyHandle,
	pub old_vel: Vector3<f32>,
}

impl Physics
{
	pub fn new(handle: rapier3d::dynamics::RigidBodyHandle) -> Self
	{
		Self {
			handle,
			old_vel: Vector3::zeros(),
		}
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Controller
{
	pub want_move: Vector3<f32>,
	pub want_rotate: Vector3<f32>,
	pub want_gripper: [bool; 2],
	pub want_fire: bool,
	pub force_attach: bool,
}

impl Controller
{
	pub fn new() -> Self
	{
		Self {
			want_move: Vector3::zeros(),
			want_rotate: Vector3::zeros(),
			want_gripper: [false, false],
			want_fire: false,
			force_attach: false,
		}
	}
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GripperStatus
{
	AttachedToParent,
	AttachedToLevel,
	Flying,
}

#[derive(Debug, Clone)]
pub struct Gripper
{
	pub parent: hecs::Entity,
	pub offset: Vector3<f32>,
	pub status: GripperStatus,
	pub time_to_grip: f64,
	pub attach_joint: Option<ImpulseJointHandle>,
	pub spring_joint: Option<ImpulseJointHandle>,
	pub connector: Option<hecs::Entity>,
}

impl Gripper
{
	pub fn new(parent: hecs::Entity, offset: Vector3<f32>) -> Self
	{
		Self {
			parent: parent,
			offset: offset,
			status: GripperStatus::AttachedToParent,
			time_to_grip: 0.,
			attach_joint: None,
			spring_joint: None,
			connector: None,
		}
	}
}

#[derive(Debug, Clone)]
pub struct Grippers
{
	pub grippers: [hecs::Entity; 2],
}

impl Grippers
{
	pub fn new(grippers: [hecs::Entity; 2]) -> Self
	{
		Self { grippers: grippers }
	}
}

pub struct Connector
{
	pub start: hecs::Entity,
	pub end: hecs::Entity,
	pub start_offset: Vector3<f32>,
	pub end_offset: Vector3<f32>,
}

impl Connector
{
	pub fn new(
		start: hecs::Entity, end: hecs::Entity, start_offset: Vector3<f32>,
		end_offset: Vector3<f32>,
	) -> Self
	{
		Self {
			start: start,
			end: end,
			start_offset: start_offset,
			end_offset: end_offset,
		}
	}
}

#[derive(Debug, Clone)]
pub enum AIState
{
	Idle,
	Attacking(hecs::Entity),
}

#[derive(Debug, Clone)]
pub struct AI
{
	pub state: AIState,
}

impl AI
{
	pub fn new() -> Self
	{
		Self {
			state: AIState::Idle,
		}
	}
}

#[derive(Debug, Clone)]
pub struct Health
{
	pub health: f32,
	pub max_health: f32,
	pub dead: bool,
	pub remove_on_death: bool,
}

impl Health
{
	pub fn new(max_health: f32) -> Self
	{
		Self {
			health: max_health,
			max_health: max_health,
			dead: false,
			remove_on_death: true,
		}
	}
}

#[derive(Debug, Clone)]
pub struct Weapon
{
	pub time_to_fire: f64,
	pub fire_delay: f64,
	pub offset: Vector3<f32>,
}

impl Weapon
{
	pub fn new(offset: Vector3<f32>) -> Self
	{
		Self {
			time_to_fire: 0.0,
			fire_delay: 1.5,
			offset: offset,
		}
	}
}

#[derive(Debug, Clone)]
pub enum Effect
{
	Die,
	Damage
	{
		amount: f32,
		owner: hecs::Entity,
	},
	GripperPierce
	{
		old_vel: Vector3<f32>,
	},
	SpawnHit,
	SpawnExplosion,
	Open,
	StartExitAnimation,
	OpenExit,
	SpawnExplosionSpawner,
}

#[derive(Debug, Clone)]
pub struct OnCollideEffects
{
	pub effects: Vec<Effect>,
}

impl OnCollideEffects
{
	pub fn new(effects: &[Effect]) -> Self
	{
		Self {
			effects: effects.to_owned(),
		}
	}
}

#[derive(Debug, Clone)]
pub struct OnDeathEffects
{
	pub effects: Vec<Effect>,
}

impl OnDeathEffects
{
	pub fn new(effects: &[Effect]) -> Self
	{
		Self {
			effects: effects.to_owned(),
		}
	}
}

#[derive(Debug, Clone)]
pub struct ExplosionScaling
{
	pub time_to_die: f64,
}

impl ExplosionScaling
{
	pub fn new(time_to_die: f64) -> Self
	{
		Self {
			time_to_die: time_to_die,
		}
	}
}

#[derive(Debug, Clone, PartialEq)]
pub enum DoorStatus
{
	Open,
	Closed,
}

#[derive(Debug, Clone)]
pub struct Door
{
	pub open_on_exit: bool,
	pub status: DoorStatus,
	pub time_to_close: f64,
	pub want_open: bool,
}

impl Door
{
	pub fn new(open_on_exit: bool) -> Self
	{
		Door {
			open_on_exit: open_on_exit,
			status: DoorStatus::Closed,
			time_to_close: 0.,
			want_open: false,
		}
	}
}

#[derive(Debug, Clone)]
pub struct SceneObjectPosition
{
	pub scene_name: String,
	pub object_idx: i32,
	pub animation_state: slhack::scene::AnimationState,
}

impl SceneObjectPosition
{
	pub fn new(
		scene_name: &str, object_idx: i32, animation_state: slhack::scene::AnimationState,
	) -> Self
	{
		Self {
			scene_name: scene_name.to_string(),
			object_idx: object_idx,
			animation_state: animation_state,
		}
	}
}

#[derive(Debug, Clone)]
pub struct PositionCopier
{
	pub target: hecs::Entity,
}

#[derive(Debug, Clone)]
pub struct ExplosionSpawner
{
	pub time_for_explosion: f64,
}

impl ExplosionSpawner
{
	pub fn new() -> Self
	{
		Self {
			time_for_explosion: 0.,
		}
	}
}
