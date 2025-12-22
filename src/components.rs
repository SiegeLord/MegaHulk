use allegro::*;
use nalgebra::{Point3, Quaternion, Unit, Vector3};
use rand::prelude::*;
use rapier3d::dynamics::ImpulseJointHandle;
use slhack::{scene, sprite};
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

	pub fn new_with_animation(scene: &str, animation: &str) -> Self
	{
		let mut animation_states = HashMap::new();
		animation_states.insert(
			0,
			AnimationState {
				speed: 1.0,
				state: scene::AnimationState::new(animation, false),
			},
		);
		Self {
			scene: scene.to_string(),
			color: Color::from_rgb_f(1., 1., 1.),
			animation_states: animation_states,
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
	// Before collisions and such.
	pub old_vel: Vector3<f32>,
	pub copy_rot: bool,
}

impl Physics
{
	pub fn new(handle: rapier3d::dynamics::RigidBodyHandle) -> Self
	{
		Self {
			handle,
			old_vel: Vector3::zeros(),
			copy_rot: true,
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
	pub want_enrage: bool,
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
			want_enrage: false,
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
	pub power_level: f32,
	pub last_kill_time: f64,
}

impl Grippers
{
	pub fn new(grippers: [hecs::Entity; 2]) -> Self
	{
		Self {
			power_level: 1.,
			grippers: grippers,
			last_kill_time: -10.,
		}
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
	pub death_effects: Vec<Effect>,
	pub recovery: f32,
	pub max_recovery: f32,

	pub damaged_by: Option<hecs::Entity>,
	pub damage_time: f64,
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
			death_effects: vec![],
			recovery: 0.,
			max_recovery: max_health / 2.,

			damaged_by: None,
			damage_time: -10.,
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

#[derive(Debug, Copy, Clone)]
pub enum ExplosionKind
{
	Small,
	Big,
	Energy,
}

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
pub enum KeyKind
{
	Red,
	Yellow,
	Blue,
}

impl KeyKind
{
	pub fn color(&self) -> Color
	{
		match self
		{
			KeyKind::Red => Color::from_rgb_f(1., 0., 0.),
			KeyKind::Yellow => Color::from_rgb_f(1., 1., 0.),
			KeyKind::Blue => Color::from_rgb_f(0., 0., 1.),
		}
	}

	pub fn from_str(key: &str) -> Option<Self>
	{
		match key
		{
			"red" => Some(Self::Red),
			"yellow" => Some(Self::Yellow),
			"blue" => Some(Self::Blue),
			_ => None,
		}
	}
}

impl std::fmt::Display for KeyKind
{
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result
	{
		match self
		{
			Self::Red => write!(f, "Red"),
			Self::Yellow => write!(f, "Yellow"),
			Self::Blue => write!(f, "Blue"),
		}
	}
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ItemKind
{
	Energy,
	Key
	{
		kind: KeyKind,
	},
	Gift,
}

impl ItemKind
{
	pub fn from_str(s: &str) -> Option<Self>
	{
		match s
		{
			"energy" => Some(Self::Energy),
			"red_key" => Some(Self::Key { kind: KeyKind::Red }),
			"blue_key" => Some(Self::Key {
				kind: KeyKind::Blue,
			}),
			"yellow_key" => Some(Self::Key {
				kind: KeyKind::Yellow,
			}),
			"gift" => Some(Self::Gift),
			_ => None,
		}
	}
}

impl std::fmt::Display for ItemKind
{
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result
	{
		match self
		{
			ItemKind::Energy => write!(f, "Energy"),
			ItemKind::Gift => write!(f, "Gift"),
			ItemKind::Key { kind } => write!(f, "{} Key", kind),
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
	SpawnExplosion
	{
		kind: ExplosionKind,
	},
	Open,
	StartExitAnimation,
	OpenExit,
	StartSelfDestruct,
	ExplosionSpawner
	{
		kind: ExplosionKind,
	},
	SpawnDeathCamera,
	DelayedDeath
	{
		delay: f64,
	},
	PickupItem
	{
		kind: ItemKind,
	},
	SpawnItem
	{
		spawn_table: Vec<(f32, ItemKind)>,
	},
	ReattachGripper,
	AddToScore
	{
		amount: i32,
	},
	ClearGifts,
	EjectInventory,
	RobotDestroyed,
	SendMessage
	{
		message: String,
	},
	AllowCinematicSkip,
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
	pub scale_rate: f32,
}

impl ExplosionScaling
{
	pub fn new(scale_rate: f32) -> Self
	{
		Self {
			scale_rate: scale_rate,
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
	pub key: Option<KeyKind>,
}

impl Door
{
	pub fn new(open_on_exit: bool, key: Option<KeyKind>) -> Self
	{
		Door {
			open_on_exit: open_on_exit,
			status: DoorStatus::Closed,
			time_to_close: 0.,
			want_open: false,
			key: key,
		}
	}
}

#[derive(Debug, Clone)]
pub struct SceneObjectPosition
{
	pub scene_name: String,
	pub object_idx: i32,
	pub animation_state: slhack::scene::AnimationState,
	pub animation_end_effects: Vec<Effect>,
}

impl SceneObjectPosition
{
	pub fn new(
		scene_name: &str, object_idx: i32, animation_state: slhack::scene::AnimationState,
		effects: &[Effect],
	) -> Self
	{
		Self {
			scene_name: scene_name.to_string(),
			object_idx: object_idx,
			animation_state: animation_state,
			animation_end_effects: effects.to_vec(),
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
	pub kind: ExplosionKind,
}

impl ExplosionSpawner
{
	pub fn new(kind: ExplosionKind) -> Self
	{
		Self {
			time_for_explosion: 0.,
			kind: kind,
		}
	}
}

#[derive(Debug, Clone)]
pub struct FaceTowards
{
	pub target: hecs::Entity,
	pub last_pos: Point3<f32>,
}

impl FaceTowards
{
	pub fn new(target: hecs::Entity) -> Self
	{
		Self {
			target: target,
			last_pos: Point3::origin(),
		}
	}
}

#[derive(Debug, Clone)]
pub struct TimeToDie
{
	pub time_to_die: f64,
	pub dead: bool,
}

impl TimeToDie
{
	pub fn new(time_to_die: f64) -> Self
	{
		Self {
			time_to_die: time_to_die,
			dead: false,
		}
	}
}

#[derive(Debug, Clone)]
pub struct StatValues
{
	pub speed: f32,
	pub rot_speed: f32,
}

impl StatValues
{
	pub fn new_player() -> Self
	{
		Self {
			speed: 16.,
			rot_speed: 4.,
		}
	}

	pub fn new_robot() -> Self
	{
		Self {
			speed: 16.,
			rot_speed: 3.,
		}
	}
}

#[derive(Debug, Clone)]
pub struct Stats
{
	pub base: StatValues,
	pub cur: StatValues,
}

impl Stats
{
	pub fn new(values: StatValues) -> Self
	{
		Self {
			base: values.clone(),
			cur: values,
		}
	}

	pub fn update(&mut self, power_level: f32)
	{
		self.cur = self.base.clone();
		self.cur.speed *= power_level;
		//self.cur.rot_speed *= power_level;
	}
}

#[derive(Debug, Clone)]
pub struct Inventory
{
	pub num_gifts: NumberTracker,
}

impl Inventory
{
	pub fn new() -> Self
	{
		Self {
			num_gifts: NumberTracker::new(0),
		}
	}
}

#[derive(Debug, Clone)]
pub struct NumberTracker
{
	pub value: i32,
	pub last_change: i32,
	pub last_time: f64,
}

impl NumberTracker
{
	pub fn new(value: i32) -> Self
	{
		Self {
			value: value,
			last_change: 0,
			last_time: 0.0,
		}
	}

	pub fn fadeout(&self, time: f64) -> f32
	{
		1. - ((time - self.last_time) / 2.).min(1.) as f32
	}

	pub fn add(&mut self, value: i32, time: f64)
	{
		self.value += value;
		if self.fadeout(time) > 0.
		{
			self.last_change += value;
		}
		else
		{
			self.last_change = value;
		}
		self.last_time = time;
	}
}

#[derive(Debug, Clone)]
pub struct Bob
{
	pub impulse: f32,
	pub phase: f64,
}

impl Bob
{
	pub fn new(impulse: f32) -> Self
	{
		Self {
			impulse: impulse,
			phase: rand::rng().random_range(0.0..2.0 * std::f64::consts::PI),
		}
	}
}
