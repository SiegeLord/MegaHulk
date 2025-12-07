use allegro::*;
use nalgebra::{Point3, Quaternion, Unit, Vector3};
use rand::prelude::*;
use slhack::sprite;

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

	pub scale: f32,
	old_scale: f32,
}

impl Position
{
	pub fn new(pos: Point3<f32>, rot: Unit<Quaternion<f32>>) -> Self
	{
		Self {
			pos,
			old_pos: pos,
			rot,
			old_rot: rot,
			scale: 1.,
			old_scale: 1.,
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

	pub fn draw_scale(&self, alpha: f32) -> f32
	{
		self.scale + alpha * (self.scale - self.old_scale)
	}
}

#[derive(Debug, Clone)]
pub struct Scene
{
	pub scene: String,
}

#[derive(Debug, Copy, Clone)]
pub struct Physics
{
	pub handle: rapier3d::dynamics::RigidBodyHandle,
}

#[derive(Debug, Copy, Clone)]
pub struct Controller
{
	pub want_move: Vector3<f32>,
	pub want_rotate: Vector3<f32>,
	pub want_gripper: [bool; 2],
}

impl Controller
{
	pub fn new() -> Self
	{
		Self {
			want_move: Vector3::zeros(),
			want_rotate: Vector3::zeros(),
			want_gripper: [false, false],
		}
	}
}

#[derive(Debug, Clone)]
pub struct Gripper
{
	pub parent: hecs::Entity,
}

impl Gripper
{
	pub fn new(parent: hecs::Entity) -> Self
	{
		Self { parent: parent }
	}
}

#[derive(Debug, Clone)]
pub struct Grippers
{
	pub time_to_grip: [f64; 2],
	pub grippers: [Option<hecs::Entity>; 2],
	pub offsets: [Vector3<f32>; 2],
}

impl Grippers
{
	pub fn new() -> Self
	{
		Self {
			time_to_grip: [0.0, 0.0],
			grippers: [None, None],
			offsets: [Vector3::new(-0.5, 0., -1.0), Vector3::new(0.5, 0., -1.0)],
		}
	}
}
