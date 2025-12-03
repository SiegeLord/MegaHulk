use crate::error::{Result, ResultHelper};
use crate::game_state::DT;
use crate::{components as comps, game_state, ui, utils};
use allegro::*;
use allegro_font::*;
use na::{
	Isometry3, Matrix4, Perspective3, Point2, Point3, Quaternion, RealField, Rotation2, Rotation3,
	Similarity3, Unit, Vector2, Vector3, Vector4,
};
use nalgebra as na;
use rand::prelude::*;
use slhack::{controls, scene, sprite, ui as slhack_ui};

use std::collections::HashMap;
use std::f32::consts::PI;

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

pub fn spawn_obj(pos: Point3<f32>, world: &mut hecs::World) -> Result<hecs::Entity>
{
	let entity = world.spawn((comps::Position::new(pos),));
	Ok(entity)
}

pub fn spawn_light(
	pos: Point3<f32>, light: comps::Light, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let entity = world.spawn((comps::Position::new(pos), light));
	Ok(entity)
}

struct Map
{
	world: hecs::World,
	camera_target: Point3<f32>,
}

impl Map
{
	fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		let mut world = hecs::World::new();
		spawn_obj(Point3::new(0., 0., 0.), &mut world)?;
		game_state::cache_scene(state, "data/test_level_sprytile.glb")?;
		state.cache_bitmap("data/level_lightmap.png")?;
		game_state::cache_scene(state, "data/sphere.glb")?;
		game_state::cache_scene(state, "data/test.obj")?;

		let level_scene = state.get_scene("data/test_level_sprytile.glb").unwrap();
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

		world.spawn((
			comps::Position::new(Point3::new(2.5, 1.5, -1.)),
			comps::Scene {
				scene: "data/sphere.glb".to_string(),
			},
		));

		world.spawn((
			comps::Position::new(Point3::new(5.5, 1.5, -2.)),
			comps::Scene {
				scene: "data/test.obj".to_string(),
			},
		));

		Ok(Self {
			world: world,
			camera_target: Point3::origin(),
		})
	}

	fn logic(&mut self, state: &mut game_state::GameState)
	-> Result<Option<game_state::NextScreen>>
	{
		let mut to_die = vec![];

		let t = -(state.hs.time() / 3.);
		self.camera_target = Point3::new(20. * t.cos() as f32 - 2., 1., 20. * t.sin() as f32 - 1.);

		// Position snapshotting.
		for (_, position) in self.world.query::<&mut comps::Position>().iter()
		{
			position.snapshot();
		}

		// Input.
		if state.controls.get_action_state(game_state::Action::Move) > 0.5
		{
			for (_, position) in self.world.query::<&mut comps::Position>().iter()
			{
				position.pos.y += 100. * DT;
			}
		}

		// Movement.
		//for (_, position) in self.world.query::<&mut comps::Position>().iter()
		//{
		//	position.pos.x += 1500. * DT;
		//	if position.pos.x > state.buffer_width()
		//	{
		//		position.pos.x %= state.buffer_width();
		//		position.snapshot();
		//	}
		//}

		// Remove dead entities
		to_die.sort();
		to_die.dedup();
		for id in to_die
		{
			//println!("died {id:?}");
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

	fn camera_pos(&self) -> Point3<f32>
	{
		Point3::new(4., 2., 0.)
	}

	fn make_camera(&self) -> Isometry3<f32>
	{
		utils::make_camera(self.camera_pos(), self.camera_target)
	}

	fn draw(&mut self, state: &mut game_state::GameState) -> Result<()>
	{
		let project = self.make_project(state);
		let camera = self.make_camera();

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

		let material_mapper =
			|_material: &scene::Material<game_state::MaterialKind>,
			 texture_name: &str|
			 -> slhack::error::Result<&Bitmap> { state.get_bitmap(texture_name).into_slhack() };

		state
			.hs
			.core
			.set_shader_sampler("lightmap", state.get_bitmap("data/level_lightmap.png")?, 1)
			.ok();
		state
			.get_scene("data/test_level_sprytile.glb")
			.unwrap()
			.draw(&state.hs.core, &state.hs.prim, material_mapper);

		for (_, (position, scene)) in self
			.world
			.query::<(&comps::Position, &comps::Scene)>()
			.iter()
		{
			let shift = Isometry3::new(position.draw_pos(state.hs.alpha).coords, Vector3::zeros())
				.to_homogeneous();

			state
				.hs
				.core
				.use_transform(&utils::mat4_to_transform(camera.to_homogeneous() * shift));
			state
				.hs
				.core
				.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift))
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
			self.camera_pos(),
		)?;

		for (_, (position, light)) in self
			.world
			.query::<(&comps::Position, &comps::Light)>()
			.iter()
		{
			let shift = Isometry3::new(position.draw_pos(state.hs.alpha).coords, Vector3::zeros());
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
