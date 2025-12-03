use crate::error::Result;
use crate::{components, game, game_state, utils};

use allegro::*;
use allegro_font::*;
use allegro_sys::*;
use nalgebra::{Matrix4, Point2, Vector2, Vector3};
use serde_derive::{Deserialize, Serialize};
use slhack::controls::Action as ControlsAction;
use slhack::{controls, ui};

pub const UNSELECTED: Color = Color::from_rgb_f(0.9, 0.9, 0.4);
pub const LABEL: Color = Color::from_rgb_f(0.7 * 0.9, 0.7 * 0.9, 0.7 * 0.4);
pub const SELECTED: Color = Color::from_rgb_f(1., 1., 1.);

pub const HORIZ_SPACE: f32 = 16.;
pub const VERT_SPACE: f32 = 16.;

const THEME: ui::Theme = ui::Theme {
	unselected: Color::from_rgb_f(0.9, 0.9, 0.4),
	label: Color::from_rgb_f(0.7 * 0.9, 0.7 * 0.9, 0.7 * 0.9),
	selected: Color::from_rgb_f(1., 1., 1.),

	horiz_space: 16.,
	vert_space: 16.,
};

pub const BUTTON_WIDTH: f32 = 128.;
pub const BUTTON_HEIGHT: f32 = 16.;
pub const CONTROL_WIDTH: f32 = 80.;

use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq)]
pub enum Action
{
	SelectMe,
	MainMenu,
	Start,
	Resume,
	Quit,
	Back,
	Forward(fn(&mut game_state::GameState) -> Result<SubScreen>),
	ToggleFullscreen,
	ToggleFracScale,
	ChangeInput(game_state::Action, usize),
	MouseSensitivity(f32),
	UiScale(f32),
	MusicVolume(f32),
	SfxVolume(f32),
}

impl ui::Action for Action
{
	const SELECT_ME: Self = Action::SelectMe;
	const BACK: Self = Action::Back;
}

pub struct MainMenu
{
	widgets: ui::WidgetList<Action>,
}

impl MainMenu
{
	pub fn new(state: &game_state::GameState) -> Result<Self>
	{
		let w = BUTTON_WIDTH;
		let h = BUTTON_HEIGHT;

		let mut widgets = vec![];

		let mut path_buf = utils::user_data_path(&state.hs.core)?;
		path_buf.push("save.cfg");
		if path_buf.exists()
		{
			widgets.push(vec![ui::Widget::Button(ui::Button::new(
				w,
				h,
				"Resume Game",
				Action::Resume,
				THEME.clone(),
			))]);
		}

		widgets.extend([
			vec![ui::Widget::Button(ui::Button::new(
				w,
				h,
				"New Game",
				Action::Start,
				THEME.clone(),
			))],
			vec![ui::Widget::Button(ui::Button::new(
				w,
				h,
				"Controls",
				Action::Forward(|s| Ok(SubScreen::ControlsMenu(ControlsMenu::new(s)))),
				THEME.clone(),
			))],
			vec![ui::Widget::Button(ui::Button::new(
				w,
				h,
				"Options",
				Action::Forward(|s| Ok(SubScreen::OptionsMenu(OptionsMenu::new(s)))),
				THEME.clone(),
			))],
			vec![ui::Widget::Button(ui::Button::new(
				w,
				h,
				"Quit",
				Action::Quit,
				THEME.clone(),
			))],
		]);

		let mut res = Self {
			widgets: ui::WidgetList::new(
				&widgets.iter().map(|r| &r[..]).collect::<Vec<_>>(),
				THEME.clone(),
			),
		};
		res.resize(state);
		Ok(res)
	}

	pub fn draw(&self, state: &game_state::GameState)
	{
		self.widgets.draw(&state.hs);

		//let sprite = "data/title.cfg";
		//let sprite = state
		//	.get_sprite(sprite)
		//	.expect(&format!("Could not find sprite: {}", sprite));
		//sprite.draw(
		//	Point2::new(state.hs.buffer_width() / 2., state.hs.buffer_height() / 2. - 125.),
		//	0,
		//	Color::from_rgb_f(1., 1., 1.),
		//	state,
		//);
		let lh = state.hs.ui_font().get_line_height() as f32;
		state.hs.core.draw_text(
			state.hs.ui_font(),
			UNSELECTED,
			HORIZ_SPACE,
			state.hs.buffer_height() - lh - VERT_SPACE,
			FontAlign::Left,
			&format!("Version: {}", game_state::VERSION),
		);
	}

	pub fn input(&mut self, state: &mut game_state::GameState, event: &Event) -> Option<Action>
	{
		self.widgets.input(event, &mut state.sfx, &mut state.hs)
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		let cx = state.hs.buffer_width() / 2.;
		let cy = state.hs.buffer_height() / 2. + 16.;

		self.widgets.set_pos(Point2::new(cx, cy));
		self.widgets.resize(&state.hs);
	}
}

pub struct ControlsMenu
{
	widgets: ui::WidgetList<Action>,
	accepting_input: bool,
}

impl ControlsMenu
{
	pub fn new(state: &game_state::GameState) -> Self
	{
		let w = CONTROL_WIDTH;
		let h = BUTTON_HEIGHT;

		let mut widgets = vec![];
		// widgets.push(vec![
		// 	ui::Widget::Label(ui::Label::new(0., 0., w * 1.5, h, "MOUSE SENSITIVITY")),
		// 	ui::Widget::Slider(ui::Slider::new(
		// 		0.,
		// 		0.,
		// 		w,
		// 		h,
		// 		state.controls.get_mouse_sensitivity(),
		// 		0.,
		// 		2.,
		// 		false,
		// 		|i| Action::MouseSensitivity(i),
		// 	)),
		// ]);

		for (&action, &inputs) in state.controls.get_actions_to_inputs()
		{
			let mut row = vec![ui::Widget::Label(ui::Label::new(
				w,
				h,
				&action.to_str(),
				THEME.clone(),
			))];
			for i in 0..2
			{
				let input = inputs[i];
				let input_str = input
					.map(|i| i.to_str().to_string())
					.unwrap_or("None".into());
				row.push(ui::Widget::Button(ui::Button::new(
					w,
					h,
					&input_str,
					Action::ChangeInput(action, i),
					THEME.clone(),
				)));
			}
			widgets.push(row);
		}
		widgets.push(vec![ui::Widget::Button(ui::Button::new(
			w,
			h,
			"Back",
			Action::Back,
			THEME.clone(),
		))]);

		let mut res = Self {
			widgets: ui::WidgetList::new(
				&widgets.iter().map(|r| &r[..]).collect::<Vec<_>>(),
				THEME.clone(),
			),
			accepting_input: false,
		};
		res.resize(state);
		res
	}

	pub fn draw(&self, state: &game_state::GameState)
	{
		self.widgets.draw(&state.hs);
	}

	pub fn input(&mut self, state: &mut game_state::GameState, event: &Event) -> Option<Action>
	{
		let mut action = None;
		let mut options_changed = false;
		if self.accepting_input
		{
			let cur_selection = self.widgets.cur_selection();
			match &mut self.widgets.widgets_mut()[cur_selection.0][cur_selection.1]
			{
				ui::Widget::Button(b) =>
				{
					if let Action::ChangeInput(action, index) = *b.get_action()
					{
						if let Some(changed) = state.controls.change_action(action, index, event)
						{
							options_changed = changed;
							state.sfx.play_sound("data/ui2.ogg").unwrap();
							self.accepting_input = false;
						}
					}
				}
				_ => (),
			}
		}
		else
		{
			if let allegro::Event::KeyDown {
				keycode: allegro::KeyCode::Delete,
				..
			} = event
			{
				let cur_selection = self.widgets.cur_selection();
				match &mut self.widgets.widgets_mut()[cur_selection.0][cur_selection.1]
				{
					ui::Widget::Button(b) =>
					{
						if let Action::ChangeInput(action, index) = *b.get_action()
						{
							state.controls.clear_action(action, index);
							options_changed = true;
							state.sfx.play_sound("data/ui2.ogg").unwrap();
						}
					}
					_ => (),
				}
			}
			action = self.widgets.input(event, &mut state.sfx, &mut state.hs);
			match action
			{
				Some(Action::ChangeInput(_, _)) =>
				{
					self.accepting_input = true;
					let cur_selection = self.widgets.cur_selection();
					match &mut self.widgets.widgets_mut()[cur_selection.0][cur_selection.1]
					{
						ui::Widget::Button(b) => b.set_text("<Input>"),
						_ => (),
					}
				}
				Some(Action::MouseSensitivity(ms)) =>
				{
					state.controls.set_mouse_sensitivity(ms);
					options_changed = true;
				}
				Some(Action::Back) =>
				{
					game_state::save_options(&state.hs.core, &state.options).unwrap();
				}
				_ => (),
			}
		}
		if options_changed
		{
			for widget_row in self.widgets.widgets_mut()
			{
				for widget in widget_row
				{
					match widget
					{
						ui::Widget::Button(b) =>
						{
							if let Action::ChangeInput(action, index) = *b.get_action()
							{
								b.set_text(
									&state.controls.get_inputs(action).unwrap()[index]
										.map(|a| a.to_str().to_string())
										.unwrap_or("None".into()),
								);
							}
						}
						_ => (),
					}
				}
			}
			state.options.controls = state.controls.get_controls().clone();
		}
		action
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		let cx = state.hs.buffer_width() / 2.;
		let cy = state.hs.buffer_height() / 2.;
		self.widgets.set_pos(Point2::new(cx, cy));
		self.widgets.resize(&state.hs);
	}
}

pub struct OptionsMenu
{
	widgets: ui::WidgetList<Action>,
}

impl OptionsMenu
{
	pub fn new(state: &game_state::GameState) -> Self
	{
		let w = BUTTON_WIDTH;
		let h = BUTTON_HEIGHT;

		let widgets = [
			vec![
				ui::Widget::Label(ui::Label::new(w, h, "Fullscreen", THEME.clone())),
				ui::Widget::Toggle(ui::Toggle::new(
					w,
					h,
					state.options.gfx.fullscreen as usize,
					vec!["No".into(), "Yes".into()],
					|_| Action::ToggleFullscreen,
					THEME.clone(),
				)),
			],
			vec![
				ui::Widget::Label(ui::Label::new(w, h, "Fractional Scale", THEME.clone())),
				ui::Widget::Toggle(ui::Toggle::new(
					w,
					h,
					state.options.gfx.frac_scale as usize,
					vec!["No".into(), "Yes".into()],
					|_| Action::ToggleFracScale,
					THEME.clone(),
				)),
			],
			vec![
				ui::Widget::Label(ui::Label::new(w, h, "Music", THEME.clone())),
				ui::Widget::Slider(ui::Slider::new(
					w,
					h,
					state.options.music_volume,
					0.,
					4.,
					0.1,
					|i| Action::MusicVolume(i),
					THEME.clone(),
				)),
			],
			vec![
				ui::Widget::Label(ui::Label::new(w, h, "SFX", THEME.clone())),
				ui::Widget::Slider(ui::Slider::new(
					w,
					h,
					state.options.sfx_volume,
					0.,
					4.,
					0.1,
					|i| Action::SfxVolume(i),
					THEME.clone(),
				)),
			],
			//vec![
			//	ui::Widget::Label(ui::Label::new(w, h, "UI Scale", THEME.clone())),
			//	ui::Widget::Slider(ui::Slider::new(
			//		w,
			//		h,
			//		state.options.ui_scale,
			//		1.,
			//		4.,
			//		0.25,
			//		|i| Action::UiScale(i), THEME.clone(),
			//	)),
			//],
			//vec![
			//	ui::Widget::Label(ui::Label::new(w, h, "Scroll", THEME.clone())),
			//	ui::Widget::Slider(ui::Slider::new(
			//		w,
			//		h,
			//		state.options.camera_speed as f32,
			//		1.,
			//		10.,
			//		1.,
			//		|i| Action::CameraSpeed(i as i32), THEME.clone(),
			//	)),
			//],
			vec![ui::Widget::Button(ui::Button::new(
				w,
				h,
				"Back",
				Action::Back,
				THEME.clone(),
			))],
		];

		let mut res = Self {
			widgets: ui::WidgetList::new(
				&widgets.iter().map(|r| &r[..]).collect::<Vec<_>>(),
				THEME.clone(),
			),
		};
		res.resize(state);
		res
	}

	pub fn draw(&self, state: &game_state::GameState)
	{
		self.widgets.draw(&state.hs);
	}

	pub fn input(&mut self, state: &mut game_state::GameState, event: &Event) -> Option<Action>
	{
		let action = self.widgets.input(event, &mut state.sfx, &mut state.hs);
		if let Some(action) = action
		{
			match action
			{
				Action::ToggleFullscreen =>
				{
					state.options.gfx.fullscreen = !state.options.gfx.fullscreen;
				}
				Action::ToggleFracScale =>
				{
					state.options.gfx.frac_scale = !state.options.gfx.frac_scale;
				}
				Action::MusicVolume(v) =>
				{
					state.options.music_volume = v;
					state.sfx.set_music_volume(v);
				}
				Action::SfxVolume(v) =>
				{
					state.options.sfx_volume = v;
					state.sfx.set_sfx_volume(v);
				}
				Action::UiScale(v) =>
				{
					state.options.gfx.ui_scale = v;
				}
				Action::Back =>
				{
					game_state::save_options(&state.hs.core, &state.options).unwrap();
					return Some(Action::Back);
				}
				_ => return Some(action),
			}
		}
		None
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		let cx = state.hs.buffer_width() / 2.;
		let cy = state.hs.buffer_height() / 2.;
		self.widgets.set_pos(Point2::new(cx, cy));
		self.widgets.resize(&state.hs);
	}
}

pub struct InGameMenu
{
	widgets: ui::WidgetList<Action>,
}

impl InGameMenu
{
	pub fn new(state: &game_state::GameState) -> Self
	{
		let w = BUTTON_WIDTH;
		let h = BUTTON_HEIGHT;

		let widgets = ui::WidgetList::new(
			&[
				&[ui::Widget::Button(ui::Button::new(
					w,
					h,
					"Resume",
					Action::Back,
					THEME.clone(),
				))],
				&[ui::Widget::Button(ui::Button::new(
					w,
					h,
					"Controls",
					Action::Forward(|s| Ok(SubScreen::ControlsMenu(ControlsMenu::new(s)))),
					THEME.clone(),
				))],
				&[ui::Widget::Button(ui::Button::new(
					w,
					h,
					"Options",
					Action::Forward(|s| Ok(SubScreen::OptionsMenu(OptionsMenu::new(s)))),
					THEME.clone(),
				))],
				&[ui::Widget::Button(ui::Button::new(
					w,
					h,
					"Save and Quit",
					Action::MainMenu,
					THEME.clone(),
				))],
			],
			THEME.clone(),
		);
		let mut res = Self { widgets };
		res.resize(state);
		res
	}

	pub fn draw(&self, state: &game_state::GameState)
	{
		self.widgets.draw(&state.hs);
	}

	pub fn input(&mut self, state: &mut game_state::GameState, event: &Event) -> Option<Action>
	{
		self.widgets.input(event, &mut state.sfx, &mut state.hs)
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		let cx = state.hs.buffer_width() / 2.;
		let cy = state.hs.buffer_height() / 2.;
		self.widgets.set_pos(Point2::new(cx, cy));
		self.widgets.resize(&state.hs);
	}
}

pub enum SubScreen
{
	MainMenu(MainMenu),
	ControlsMenu(ControlsMenu),
	OptionsMenu(OptionsMenu),
	InGameMenu(InGameMenu),
}

impl SubScreen
{
	pub fn draw(&self, state: &game_state::GameState)
	{
		match self
		{
			SubScreen::MainMenu(s) => s.draw(state),
			SubScreen::ControlsMenu(s) => s.draw(state),
			SubScreen::OptionsMenu(s) => s.draw(state),
			SubScreen::InGameMenu(s) => s.draw(state),
		}
	}

	pub fn input(&mut self, state: &mut game_state::GameState, event: &Event) -> Option<Action>
	{
		match self
		{
			SubScreen::MainMenu(s) => s.input(state, event),
			SubScreen::ControlsMenu(s) => s.input(state, event),
			SubScreen::OptionsMenu(s) => s.input(state, event),
			SubScreen::InGameMenu(s) => s.input(state, event),
		}
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		match self
		{
			SubScreen::MainMenu(s) => s.resize(state),
			SubScreen::ControlsMenu(s) => s.resize(state),
			SubScreen::OptionsMenu(s) => s.resize(state),
			SubScreen::InGameMenu(s) => s.resize(state),
		}
	}
}

pub struct SubScreens
{
	pub subscreens: Vec<SubScreen>,
	pub action: Option<Action>,
	pub time_to_transition: f64,
}

const TRANSITION_TIME: f64 = 0.25;

impl SubScreens
{
	pub fn new(state: &game_state::GameState) -> Self
	{
		Self {
			subscreens: vec![],
			action: None,
			time_to_transition: state.hs.core.get_time(),
		}
	}

	pub fn reset_transition(&mut self, state: &game_state::GameState)
	{
		self.time_to_transition = state.hs.core.get_time();
	}

	pub fn draw(&self, state: &game_state::GameState)
	{
		let time = state.hs.core.get_time();
		let f = if self.time_to_transition > time
		{
			-1. + (self.time_to_transition - time) / TRANSITION_TIME
		}
		else
		{
			(1. - (time - self.time_to_transition) / TRANSITION_TIME).max(0.)
		};
		let f = f as f32;
		let mut transform = Transform::identity();
		transform.translate(0., state.hs.buffer_height() * f);
		state.hs.core.use_transform(&transform);
		if let Some(subscreen) = self.subscreens.last()
		{
			subscreen.draw(state);
		}
		state.hs.core.use_transform(&Transform::identity());
	}

	pub fn input(
		&mut self, state: &mut game_state::GameState, event: &Event,
	) -> Result<Option<Action>>
	{
		if self.action.is_none()
		{
			self.action = self.subscreens.last_mut().unwrap().input(state, event);
			let is_change_input = if let Some(Action::ChangeInput(_, _)) = self.action
			{
				true
			}
			else
			{
				false
			};
			if self.action.is_some() && self.action != Some(Action::SelectMe) && !is_change_input
			{
				self.time_to_transition = state.hs.core.get_time() + TRANSITION_TIME;
			}
		}
		if let (Some(action), true) = (
			self.action.clone(),
			state.hs.core.get_time() > self.time_to_transition,
		)
		{
			self.action = None;
			match action
			{
				Action::Forward(subscreen_fn) =>
				{
					self.subscreens.push(subscreen_fn(state)?);
				}
				Action::Back =>
				{
					self.subscreens.pop().unwrap();
				}
				action @ _ => return Ok(Some(action)),
			}
		}
		Ok(None)
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		for subscreen in &mut self.subscreens
		{
			subscreen.resize(state);
		}
	}

	pub fn pop(&mut self)
	{
		self.subscreens.pop();
	}

	pub fn push(&mut self, screen: SubScreen)
	{
		self.subscreens.push(screen);
	}

	pub fn is_empty(&self) -> bool
	{
		self.subscreens.is_empty()
	}
}
