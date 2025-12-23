use crate::error::Result;
use crate::game::MapStats;
use crate::{components, game_state, ui, utils};

use allegro::*;
use allegro_font::*;
use allegro_sys::*;
use nalgebra::{Matrix4, Point2};
use rand::prelude::*;
use slhack::controls;

pub struct Intermission
{
	intermission_menu: ui::IntermissionMenu,
}

impl Intermission
{
	pub fn new(map_stats: &MapStats, state: &mut game_state::GameState) -> Result<Self>
	{
		state.hs.paused = false;
		state.hs.hide_mouse = true;
		state.sfx.cache_sample("data/ui1.ogg")?;
		state.sfx.cache_sample("data/ui2.ogg")?;
		state
			.sfx
			.play_music("data/MegaHulk_Intermission.ogg", 0.5, &state.hs.core);

		let intermission_menu = ui::IntermissionMenu::new(map_stats, state)?;
		Ok(Self { intermission_menu })
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
		if let Some(action) = self.intermission_menu.input(state, event)
		{
			match action
			{
				ui::Action::Start => return Ok(Some(game_state::NextScreen::Game)),
				_ => (),
			}
		}
		Ok(None)
	}

	pub fn draw(&mut self, state: &game_state::GameState) -> Result<()>
	{
		// TODO: Why do I need to set these?
		state.hs.core.set_target_bitmap(Some(state.hs.buffer1()));
		state
			.hs
			.core
			.use_shader(Some(state.basic_shader.as_ref().unwrap()))
			.unwrap();
		state
			.hs
			.core
			.set_blender(BlendOperation::Add, BlendMode::One, BlendMode::InverseAlpha);
		state
			.hs
			.core
			.set_shader_uniform(
				"tint",
				&[crate::game::color_to_array(Color::from_rgb_f(1., 1., 1.))][..],
			)
			.ok();

		state.hs.core.clear_to_color(Color::from_rgb_f(0., 0., 0.5));
		self.intermission_menu.draw(state);
		Ok(())
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		self.intermission_menu.resize(state);
	}
}
