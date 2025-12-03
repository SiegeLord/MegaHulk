use crate::error::Result;
use crate::{ui, utils};
use allegro::*;
use allegro_font::*;
use allegro_image::*;
use allegro_primitives::*;
use allegro_ttf::*;
use nalgebra::Point2;
use serde_derive::{Deserialize, Serialize};
use slhack::hack_state::HackState;
use slhack::{atlas, controls, deferred, hack_state, scene, sfx, sprite};
use std::collections::hash_map::Entry;
use std::collections::{BTreeMap, HashMap};
use std::{fmt, path, sync};

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const DT: f32 = 1. / 60.;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(i32)]
pub enum MaterialKind
{
	Static = 0,
	Dynamic = 1,
	Fullbright = 2,
}

impl Into<i32> for MaterialKind
{
	fn into(self) -> i32
	{
		self as i32
	}
}

#[derive(PartialEq, Eq, Hash, Serialize, Deserialize, Copy, Clone, Debug, PartialOrd, Ord)]
pub enum Action
{
	Move,
}

impl controls::Action for Action
{
	fn to_str(&self) -> &'static str
	{
		match self
		{
			Action::Move => "Move",
		}
	}
}

pub fn new_game_controls() -> controls::Controls<Action>
{
	let mut action_to_inputs = BTreeMap::new();
	action_to_inputs.insert(
		Action::Move,
		[
			Some(controls::Input::Keyboard(allegro::KeyCode::Space)),
			None,
		],
	);
	controls::Controls::new(action_to_inputs)
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Options
{
	pub gfx: hack_state::GfxOptions,

	pub play_music: bool,
	pub sfx_volume: f32,
	pub music_volume: f32,
	pub camera_speed: f32,
	pub controls: controls::Controls<Action>,
}

impl Default for Options
{
	fn default() -> Self
	{
		Self {
			gfx: hack_state::GfxOptions {
				fullscreen: true,
				width: 960,
				height: 864,
				vsync_method: if cfg!(target_os = "windows") { 1 } else { 2 },
				grab_mouse: false,
				ui_scale: 1.,
				frac_scale: true,
			},
			play_music: true,
			sfx_volume: 1.,
			music_volume: 1.,
			camera_speed: 2.,
			controls: new_game_controls(),
		}
	}
}

type Scene = scene::Scene<MaterialKind>;

#[derive(Debug)]
pub enum NextScreen
{
	Game,
	Menu,
	InGameMenu,
	Quit,
}

pub struct GameState
{
	pub sfx: sfx::Sfx,
	pub atlas: atlas::Atlas,
	pub options: Options,
	pub controls: controls::ControlsHandler<Action>,

	pub basic_shader: Option<Shader>,
	pub forward_shader: Option<Shader>,
	pub light_shader: Option<Shader>,
	pub final_shader: Option<Shader>,
	pub deferred_renderer: Option<deferred::DeferredRenderer>,

	bitmaps: HashMap<String, Bitmap>,
	sprites: HashMap<String, sprite::Sprite>,
	scenes: HashMap<String, Scene>,

	// Has to be last!
	pub hs: hack_state::HackState,
}

pub fn load_options(core: &Core) -> Result<Options>
{
	Ok(utils::load_user_data(core, "options.cfg")?.unwrap_or_default())
}

pub fn save_options(core: &Core, options: &Options) -> Result<()>
{
	Ok(utils::save_user_data(core, "options.cfg", options)?)
}

impl GameState
{
	pub fn new() -> Result<Self>
	{
		let mut options = Options::default();
		let hack_load_options = |core: &Core| -> slhack::error::Result<hack_state::GfxOptions> {
			options = load_options(core).map_err(Into::<slhack::error::Error>::into)?;
			Ok(options.gfx.clone())
		};
		let hack_state =
			hack_state::HackState::new("Example3D", hack_load_options, Some((640, 480)))?;

		let sfx = sfx::Sfx::new(options.sfx_volume, options.music_volume, &hack_state.core)?;
		//sfx.set_music_file("data/lemonade-sinus.xm");
		//sfx.play_music()?;

		let controls = controls::ControlsHandler::new(options.controls.clone());
		Ok(Self {
			options: options,
			bitmaps: HashMap::new(),
			sprites: HashMap::new(),
			scenes: HashMap::new(),
			sfx: sfx,
			atlas: atlas::Atlas::new(1024),
			controls: controls,
			basic_shader: None,
			forward_shader: None,
			light_shader: None,
			final_shader: None,
			deferred_renderer: None,
			hs: hack_state,
		})
	}

	pub fn resize_display(&mut self) -> Result<()>
	{
		self.hs.resize_display().map_err(Into::into)
	}

	pub fn cache_bitmap<'l>(&'l mut self, name: &str) -> Result<&'l Bitmap>
	{
		Ok(match self.bitmaps.entry(name.to_string())
		{
			Entry::Occupied(o) => o.into_mut(),
			Entry::Vacant(v) => v.insert(utils::load_bitmap(&self.hs.core, name)?),
		})
	}

	pub fn cache_sprite<'l>(&'l mut self, name: &str) -> Result<&'l sprite::Sprite>
	{
		Ok(match self.sprites.entry(name.to_string())
		{
			Entry::Occupied(o) => o.into_mut(),
			Entry::Vacant(v) =>
			{
				v.insert(sprite::Sprite::load(name, &self.hs.core, &mut self.atlas)?)
			}
		})
	}

	fn cache_scene<'l>(&'l mut self, name: &str) -> Result<&'l Scene>
	{
		let scene = match self.scenes.entry(name.to_string())
		{
			Entry::Occupied(o) => o.into_mut(),
			Entry::Vacant(v) => v.insert(Scene::load(
				&mut self.hs.display.as_mut().unwrap(),
				&self.hs.prim,
				name,
			)?),
		};
		Ok(scene)
	}

	pub fn insert_scene(&mut self, name: &str, scene: Scene)
	{
		self.scenes.insert(name.to_string(), scene);
	}

	pub fn get_bitmap<'l>(&'l self, name: &str) -> Result<&'l Bitmap>
	{
		Ok(self
			.bitmaps
			.get(name)
			.ok_or_else(|| format!("{name} is not cached!"))?)
	}

	pub fn get_sprite<'l>(&'l self, name: &str) -> Result<&'l sprite::Sprite>
	{
		Ok(self
			.sprites
			.get(name)
			.ok_or_else(|| format!("{name} is not cached!"))?)
	}

	pub fn get_scene<'l>(&'l self, name: &str) -> Result<&'l Scene>
	{
		Ok(self
			.scenes
			.get(name)
			.ok_or_else(|| format!("{name} is not cached!"))?)
	}
}

pub fn cache_scene(state: &mut GameState, name: &str) -> Result<()>
{
	let scene = state.cache_scene(name)?;
	let mut textures = vec![];
	for object in &scene.objects
	{
		if let scene::ObjectKind::MultiMesh { meshes } = &object.kind
		{
			for mesh in meshes
			{
				if let Some(material) = mesh.material.as_ref()
				{
					textures.push(material.desc.texture.clone());
					if !material.desc.lightmap.is_empty()
					{
						textures.push(material.desc.lightmap.clone());
					}
				}
			}
		}
	}
	for texture in textures
	{
		state.cache_bitmap(&texture)?;
	}
	Ok(())
}
