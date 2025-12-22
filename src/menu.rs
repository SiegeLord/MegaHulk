use crate::error::Result;
use crate::{components, game_state, ui, utils};

use allegro::*;
use allegro_font::*;
use allegro_sys::*;
use nalgebra::{Matrix4, Point2};
use rand::prelude::*;
use slhack::controls;

pub struct Menu
{
	subscreens: ui::SubScreens,
	ignore_first_mouse_up: bool,
}

impl Menu
{
	pub fn new(ignore_first_mouse_up: bool, state: &mut game_state::GameState) -> Result<Self>
	{
		state.hs.paused = false;
		state.hs.hide_mouse = false;
		state.sfx.cache_sample("data/ui1.ogg")?;
		state.sfx.cache_sample("data/ui2.ogg")?;
		state.cache_sprite("data/title.cfg")?;

		let mut subscreens = ui::SubScreens::new(state);
		subscreens.push(ui::SubScreen::MainMenu(ui::MainMenu::new(state)?));

		Ok(Self {
			ignore_first_mouse_up: ignore_first_mouse_up,
			subscreens,
		})
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
			Event::MouseButtonUp { .. } =>
			{
				// HACK
				if self.ignore_first_mouse_up
				{
					self.ignore_first_mouse_up = false;
					return Ok(None);
				}
			}
			_ => (),
		}
		if let Some(action) = self.subscreens.input(state, event)?
		{
			match action
			{
				ui::Action::Start => return Ok(Some(game_state::NextScreen::Game)),
				ui::Action::Quit => return Ok(Some(game_state::NextScreen::Quit)),
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
		self.subscreens.draw(state);

		Ok(())
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		self.subscreens.resize(state);
	}
}
