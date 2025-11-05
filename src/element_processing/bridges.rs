use crate::args::Args;
use crate::block_definitions::*;
use crate::bresenham::bresenham_line;
use crate::element_processing::elevation::infer_structure_elevation;
use crate::osm_parser::ProcessedWay;
use crate::world_editor::WorldEditor;

const DEFAULT_BRIDGE_CLEARANCE: i32 = 5;
const DEFAULT_WALKWAY_WIDTH: i32 = 4;

pub fn generate_bridges(editor: &mut WorldEditor, way: &ProcessedWay, args: &Args) {
    if way.nodes.len() < 2 {
        return;
    }

    let has_bridge = way
        .tags
        .get("bridge")
        .map(|value| value != "no" && !value.is_empty())
        .unwrap_or(false);

    if !has_bridge {
        return;
    }

    let mut bridge_points: Vec<(i32, i32, i32)> = Vec::new();

    for segment in way.nodes.windows(2) {
        let start = &segment[0];
        let end = &segment[1];
        let segment_points = bresenham_line(start.x, 0, start.z, end.x, 0, end.z);

        for point in segment_points {
            if bridge_points
                .last()
                .map(|last_point| *last_point != point)
                .unwrap_or(true)
            {
                bridge_points.push(point);
            }
        }
    }

    if bridge_points.is_empty() {
        return;
    }

    let clearance = infer_structure_elevation(&way.tags, DEFAULT_BRIDGE_CLEARANCE).max(1);

    let mut ground_samples: Vec<(i32, i32, i32)> = Vec::with_capacity(bridge_points.len());
    let mut max_ground: i32 = i32::MIN;
    for (x, _, z) in &bridge_points {
        let ground_level = editor.get_absolute_y(*x, 0, *z);
        max_ground = max_ground.max(ground_level);
        ground_samples.push((*x, *z, ground_level));
    }

    if max_ground == i32::MIN {
        return;
    }

    let deck_y = max_ground + clearance;

    let walkway_half_width = derive_half_width(&way.tags, args.scale);
    let deck_half_width = walkway_half_width + 1; // Extend deck to cover railings
    let support_interval = derive_support_interval(args.scale);

    for (idx, ((x, _, z), (_, _, ground_level))) in
        bridge_points.iter().zip(ground_samples.iter()).enumerate()
    {
        for dx in -deck_half_width..=deck_half_width {
            editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x + dx, deck_y, *z, None, None);
            if deck_y > ground_level {
                editor.set_block_absolute(STONE_BRICKS, x + dx, deck_y - 1, *z, None, None);
            }
        }

        for railing_offset in [-deck_half_width, deck_half_width] {
            editor.set_block_absolute(
                STONE_BRICK_WALL,
                x + railing_offset,
                deck_y + 1,
                *z,
                None,
                None,
            );
        }

        if deck_y - ground_level > 1 && idx % support_interval == 0 {
            let mut pillar_y = ground_level + 1;
            while pillar_y < deck_y {
                editor.set_block_absolute(STONE_BRICKS, *x, pillar_y, *z, None, None);
                pillar_y += 1;
            }
        }
    }
}

fn derive_half_width(tags: &std::collections::HashMap<String, String>, scale: f64) -> i32 {
    let width_value = tags
        .get("bridge:width")
        .or_else(|| tags.get("width"))
        .and_then(|value| value.parse::<f32>().ok())
        .map(|width| (width * scale.max(0.5)) as i32);

    width_value
        .map(|value| (value / 2).max(2))
        .unwrap_or(DEFAULT_WALKWAY_WIDTH / 2)
}

fn derive_support_interval(scale: f64) -> usize {
    let interval = (6.0 / scale.max(0.5)).round() as i32;
    interval.clamp(2, 8) as usize
}

#[cfg(test)]
mod tests {
    use super::generate_bridges;
    use crate::args::Args;
    use crate::block_definitions::{LIGHT_GRAY_CONCRETE, STONE_BRICKS, STONE_BRICK_WALL};
    use crate::coordinate_system::{cartesian::XZBBox, geographic::LLBBox};
    use crate::ground::Ground;
    use crate::osm_parser::{ProcessedNode, ProcessedWay};
    use crate::world_editor::WorldEditor;
    use std::collections::HashMap;

    fn build_editor<'a>() -> (
        tempfile::TempDir,
        XZBBox,
        LLBBox,
        Ground,
        WorldEditor<'a>,
        Args,
    ) {
        let tmpdir = tempfile::tempdir().unwrap();
        std::fs::create_dir(tmpdir.path().join("region")).unwrap();

        let xzbbox = XZBBox::rect_from_xz_lengths(32.0, 32.0).unwrap();
        let llbbox = LLBBox::new(0.0, 0.0, 1.0, 1.0).unwrap();
        let ground = Ground::new_flat(0);

        let mut editor = WorldEditor::new(tmpdir.path().to_path_buf(), &xzbbox, llbbox);
        editor.set_ground(&ground);

        let args = Args {
            bbox: llbbox,
            file: None,
            save_json_file: None,
            path: tmpdir.path().to_path_buf(),
            downloader: "requests".to_string(),
            scale: 1.0,
            ground_level: 0,
            terrain: false,
            interior: true,
            roof: true,
            fillground: false,
            debug: false,
            timeout: None,
            spawn_point: None,
        };

        (tmpdir, xzbbox, llbbox, ground, editor, args)
    }

    fn sample_bridge_way() -> ProcessedWay {
        let nodes = (0..=4)
            .map(|idx| ProcessedNode {
                id: idx,
                tags: HashMap::new(),
                x: 8 + idx,
                z: 8,
            })
            .collect();

        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("layer".to_string(), "1".to_string());

        ProcessedWay { id: 1, nodes, tags }
    }

    #[test]
    fn bridge_places_elevated_deck_and_supports() {
        let (_tmpdir, _xzbbox, _llbbox, ground, mut editor, args) = build_editor();
        let way = sample_bridge_way();

        // Re-set ground because build_editor moved it into the tuple
        editor.set_ground(&ground);

        generate_bridges(&mut editor, &way, &args);

        let midpoint = &way.nodes[2];
        let ground_y = editor.get_absolute_y(midpoint.x, 0, midpoint.z);

        let mut deck_y = None;
        for y in ground_y + 1..ground_y + 20 {
            if editor.check_for_block_absolute(
                midpoint.x,
                y,
                midpoint.z,
                Some(&[LIGHT_GRAY_CONCRETE]),
                None,
            ) {
                deck_y = Some(y);
                break;
            }
        }

        let deck_y = deck_y.expect("Bridge deck not placed");
        assert!(deck_y > ground_y);

        // Railings on either side of the deck
        let railing_positions = [midpoint.x - 3, midpoint.x + 3];
        assert!(railing_positions.iter().any(|x| {
            editor.check_for_block_absolute(
                *x,
                deck_y + 1,
                midpoint.z,
                Some(&[STONE_BRICK_WALL]),
                None,
            )
        }));

        // Stone support column beneath the deck
        assert!(editor.check_for_block_absolute(
            midpoint.x,
            ground_y + 1,
            midpoint.z,
            Some(&[STONE_BRICKS]),
            None
        ));
    }
}
