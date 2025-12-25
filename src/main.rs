#![allow(non_snake_case)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(clippy::all)]
#![allow(unpredictable_function_pointer_comparisons)]

mod components;
mod error;
mod game;
mod game_state;
mod intermission;
mod menu;
mod ui;

use crate::error::{Result, ResultHelper};
use allegro::*;
use allegro_dialog::*;
use slhack::{deferred, game_loop, hack_state, utils};

pub enum Screen
{
	Game(game::Game),
	Menu(menu::Menu),
	Intermission(intermission::Intermission),
}

pub struct LoopState
{
	next_screen: Option<game_state::NextScreen>,
	cur_screen: Option<Screen>,
	// Must be last!
	game_state: game_state::GameState,
}

impl LoopState
{
	fn new() -> Result<Self>
	{
		Ok(Self {
			game_state: game_state::GameState::new()?,
			next_screen: None,
			cur_screen: None,
		})
	}
}

impl game_loop::LoopState for LoopState
{
	fn hs(&mut self) -> &mut hack_state::HackState
	{
		&mut self.game_state.hs
	}

	fn gfx_options(&self) -> &hack_state::GfxOptions
	{
		&self.game_state.options.gfx
	}

	fn init(&mut self) -> slhack::error::Result<()>
	{
		let game_state = &mut self.game_state;
		let hs = &mut game_state.hs;
		let replacements = game_state::shader_replacements();
		game_state.basic_shader = Some(utils::load_shader(
			hs.display_mut(),
			"data/basic",
			&replacements,
		)?);
		game_state.forward_shader = Some(utils::load_shader(
			hs.display_mut(),
			"data/forward",
			&replacements,
		)?);
		game_state.forward2_shader = Some(utils::load_shader(
			hs.display_mut(),
			"data/forward2",
			&replacements,
		)?);
		game_state.light_shader = Some(utils::load_shader(
			hs.display_mut(),
			"data/light",
			&replacements,
		)?);
		game_state.final_shader = Some(utils::load_shader(
			hs.display_mut(),
			"data/final",
			&replacements,
		)?);
		game_state.map_depth_shader = Some(utils::load_shader(
			hs.display_mut(),
			"data/map_depth",
			&replacements,
		)?);
		game_state.exploration_shader = Some(utils::load_shader(
			hs.display_mut(),
			"data/exploration",
			&replacements,
		)?);
		let (width, height) = hs.fixed_buffer_size.unwrap();
		game_state.deferred_renderer = Some(deferred::DeferredRenderer::new(
			hs.display.as_mut().unwrap(),
			&hs.prim,
			width,
			height,
		)?);
		game_state.resize_display().into_slhack()?;

		game_state.next_level_desc = Some("data/level3.cfg".to_string());
		self.cur_screen = Some(Screen::Menu(
			menu::Menu::new(false, game_state).into_slhack()?,
		));
		//self.cur_screen = Some(Screen::Game(game::Game::new(game_state).into_slhack()?));
		Ok(())
	}

	fn resize_display(&mut self) -> slhack::error::Result<()>
	{
		self.game_state.resize_display().into_slhack()?;
		match &mut self.cur_screen
		{
			Some(Screen::Menu(menu)) => menu.resize(&mut self.game_state),
			Some(Screen::Game(game)) => game.resize(&mut self.game_state),
			Some(Screen::Intermission(inter)) => inter.resize(&mut self.game_state),
			_ => (),
		}
		Ok(())
	}

	fn draw(&mut self) -> slhack::error::Result<()>
	{
		match &mut self.cur_screen
		{
			Some(Screen::Menu(menu)) => menu.draw(&mut self.game_state),
			Some(Screen::Game(game)) => game.draw(&mut self.game_state),
			Some(Screen::Intermission(inter)) => inter.draw(&mut self.game_state),
			_ => Ok(()),
		}
		.into_slhack()
	}

	fn input(&mut self, event: &Event) -> slhack::error::Result<()>
	{
		self.game_state.controls.decode_event(event);
		self.next_screen = match &mut self.cur_screen
		{
			Some(Screen::Menu(menu)) => menu.input(event, &mut self.game_state),
			Some(Screen::Game(game)) => game.input(event, &mut self.game_state),
			Some(Screen::Intermission(inter)) => inter.input(event, &mut self.game_state),
			_ => Ok(None),
		}
		.into_slhack()?;
		Ok(())
	}

	fn logic(&mut self) -> slhack::error::Result<()>
	{
		if self.next_screen.is_none()
		{
			self.next_screen = match &mut self.cur_screen
			{
				Some(Screen::Game(game)) => game.logic(&mut self.game_state),
				_ => Ok(None),
			}
			.into_slhack()?;
		}
		self.game_state.sfx.update_sounds(&self.game_state.hs.core)
	}

	fn step(&mut self) -> slhack::error::Result<bool>
	{
		match self.next_screen
		{
			Some(game_state::NextScreen::Game) =>
			{
				self.cur_screen = Some(Screen::Game(
					game::Game::new(&mut self.game_state).into_slhack()?,
				))
			}
			Some(game_state::NextScreen::Menu {
				ignore_first_mouse_up,
			}) =>
			{
				self.cur_screen = Some(Screen::Menu(
					menu::Menu::new(ignore_first_mouse_up, &mut self.game_state).into_slhack()?,
				))
			}
			Some(game_state::NextScreen::Intermission { ref map_stats }) =>
			{
				self.cur_screen = Some(Screen::Intermission(
					intermission::Intermission::new(&map_stats, &mut self.game_state)
						.into_slhack()?,
				))
			}
			Some(game_state::NextScreen::Quit) => self.cur_screen = None,
			None => (),
			_ => panic!("Unknown next screen {:?}", self.next_screen),
		};
		self.next_screen = None;
		Ok(self.cur_screen.is_some())
	}
}

fn real_main() -> Result<()>
{
	let mut state = LoopState::new()?;

	let mut options = game_loop::Options::new();
	options.depth_buffer = true;
	options.dt = game_state::DT as f64;
	game_loop::game_loop(&mut state, options)?;

	state.game_state.sfx.fade_out(&state.game_state.hs.core);
	Ok(())
}

allegro_main! {
	use std::panic::catch_unwind;

	match catch_unwind(|| real_main().unwrap())
	{
		Err(e) =>
		{
			let err: String = e
				.downcast_ref::<&'static str>()
				.map(|&e| e.to_owned())
				.or_else(|| e.downcast_ref::<String>().map(|e| e.clone()))
				.unwrap_or("Unknown error!".to_owned());

			let mut lines = vec![];
			for line in err.lines().take(10)
			{
				lines.push(line.to_string());
			}
			show_native_message_box(
				None,
				"Error!",
				"An error has occurred!",
				&lines.join("\n"),
				Some("You make me sad."),
				MESSAGEBOX_ERROR,
			);
		}
		Ok(_) => (),
	}
}
