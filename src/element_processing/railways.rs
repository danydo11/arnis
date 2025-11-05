use crate::block_definitions::*;
use crate::bresenham::bresenham_line;
use crate::element_processing::elevation::infer_structure_elevation;
use crate::osm_parser::ProcessedWay;
use crate::world_editor::WorldEditor;

const DEFAULT_RAILWAY_CLEARANCE: i32 = 4;
const SUPPORT_INTERVAL: usize = 8;

pub fn generate_railways(editor: &mut WorldEditor, element: &ProcessedWay) {
    if let Some(railway_type) = element.tags.get("railway") {
        if [
            "proposed",
            "abandoned",
            "subway",
            "construction",
            "razed",
            "turntable",
        ]
        .contains(&railway_type.as_str())
        {
            return;
        }

        if let Some(subway) = element.tags.get("subway") {
            if subway == "yes" {
                return;
            }
        }

        if let Some(tunnel) = element.tags.get("tunnel") {
            if tunnel == "yes" {
                return;
            }
        }

        let elevation_offset = infer_structure_elevation(&element.tags, DEFAULT_RAILWAY_CLEARANCE);
        let add_supports = elevation_offset > 0;

        for i in 1..element.nodes.len() {
            let prev_node = element.nodes[i - 1].xz();
            let cur_node = element.nodes[i].xz();

            let points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
            let smoothed_points = smooth_diagonal_rails(&points);

            for j in 0..smoothed_points.len() {
                let (bx, _, bz) = smoothed_points[j];

                let ground_y = editor.get_absolute_y(bx, 0, bz);
                let ballast_y = ground_y + elevation_offset;
                let rail_y = ballast_y + 1;

                editor.set_block_absolute(GRAVEL, bx, ballast_y, bz, None, None);

                let prev = if j > 0 {
                    Some(smoothed_points[j - 1])
                } else {
                    None
                };
                let next = if j < smoothed_points.len() - 1 {
                    Some(smoothed_points[j + 1])
                } else {
                    None
                };

                let rail_block = determine_rail_direction(
                    (bx, bz),
                    prev.map(|(x, _, z)| (x, z)),
                    next.map(|(x, _, z)| (x, z)),
                );

                editor.set_block_absolute(rail_block, bx, rail_y, bz, None, None);

                if bx % 4 == 0 {
                    editor.set_block_absolute(OAK_LOG, bx, ballast_y, bz, None, None);
                }

                if add_supports {
                    if elevation_offset > 1 {
                        editor.set_block_absolute(STONE_BRICKS, bx, ballast_y - 1, bz, None, None);
                    }

                    if ((bx + bz).rem_euclid(SUPPORT_INTERVAL as i32) == 0)
                        && ballast_y - ground_y > 1
                    {
                        let mut support_y = ground_y + 1;
                        while support_y < ballast_y {
                            editor.set_block_absolute(STONE_BRICKS, bx, support_y, bz, None, None);
                            support_y += 1;
                        }
                    }
                }
            }
        }
    }
}

fn smooth_diagonal_rails(points: &[(i32, i32, i32)]) -> Vec<(i32, i32, i32)> {
    let mut smoothed = Vec::new();

    for i in 0..points.len() {
        let current = points[i];
        smoothed.push(current);

        if i + 1 >= points.len() {
            continue;
        }

        let next = points[i + 1];
        let (x1, y1, z1) = current;
        let (x2, _, z2) = next;

        // If points are diagonally adjacent
        if (x2 - x1).abs() == 1 && (z2 - z1).abs() == 1 {
            // Look ahead to determine best intermediate point
            let look_ahead = if i + 2 < points.len() {
                Some(points[i + 2])
            } else {
                None
            };

            // Look behind
            let look_behind = if i > 0 { Some(points[i - 1]) } else { None };

            // Choose intermediate point based on the overall curve direction
            let intermediate = if let Some((prev_x, _, _prev_z)) = look_behind {
                if prev_x == x1 {
                    // Coming from vertical, keep x constant
                    (x1, y1, z2)
                } else {
                    // Coming from horizontal, keep z constant
                    (x2, y1, z1)
                }
            } else if let Some((next_x, _, _next_z)) = look_ahead {
                if next_x == x2 {
                    // Going to vertical, keep x constant
                    (x2, y1, z1)
                } else {
                    // Going to horizontal, keep z constant
                    (x1, y1, z2)
                }
            } else {
                // Default to horizontal first if no context
                (x2, y1, z1)
            };

            smoothed.push(intermediate);
        }
    }

    smoothed
}

fn determine_rail_direction(
    current: (i32, i32),
    prev: Option<(i32, i32)>,
    next: Option<(i32, i32)>,
) -> Block {
    let (x, z) = current;

    match (prev, next) {
        (Some((px, pz)), Some((nx, nz))) => {
            if px == nx {
                RAIL_NORTH_SOUTH
            } else if pz == nz {
                RAIL_EAST_WEST
            } else {
                // Calculate relative movements
                let from_prev = (px - x, pz - z);
                let to_next = (nx - x, nz - z);

                match (from_prev, to_next) {
                    // East to North or North to East
                    ((-1, 0), (0, -1)) | ((0, -1), (-1, 0)) => RAIL_NORTH_WEST,
                    // West to North or North to West
                    ((1, 0), (0, -1)) | ((0, -1), (1, 0)) => RAIL_NORTH_EAST,
                    // East to South or South to East
                    ((-1, 0), (0, 1)) | ((0, 1), (-1, 0)) => RAIL_SOUTH_WEST,
                    // West to South or South to West
                    ((1, 0), (0, 1)) | ((0, 1), (1, 0)) => RAIL_SOUTH_EAST,
                    _ => {
                        if (px - x).abs() > (pz - z).abs() {
                            RAIL_EAST_WEST
                        } else {
                            RAIL_NORTH_SOUTH
                        }
                    }
                }
            }
        }
        (Some((px, pz)), None) | (None, Some((px, pz))) => {
            if px == x {
                RAIL_NORTH_SOUTH
            } else if pz == z {
                RAIL_EAST_WEST
            } else {
                RAIL_NORTH_SOUTH
            }
        }
        (None, None) => RAIL_NORTH_SOUTH,
    }
}

pub fn generate_roller_coaster(editor: &mut WorldEditor, element: &ProcessedWay) {
    if let Some(roller_coaster) = element.tags.get("roller_coaster") {
        if roller_coaster == "track" {
            // Check if it's indoor (skip if yes)
            if let Some(indoor) = element.tags.get("indoor") {
                if indoor == "yes" {
                    return;
                }
            }

            // Check if layer is negative (skip if yes)
            if let Some(layer) = element.tags.get("layer") {
                if let Ok(layer_value) = layer.parse::<i32>() {
                    if layer_value < 0 {
                        return;
                    }
                }
            }

            let elevation_height = 4; // 4 blocks in the air
            let pillar_interval = 6; // Support pillars every 6 blocks

            for i in 1..element.nodes.len() {
                let prev_node = element.nodes[i - 1].xz();
                let cur_node = element.nodes[i].xz();

                let points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
                let smoothed_points = smooth_diagonal_rails(&points);

                for j in 0..smoothed_points.len() {
                    let (bx, _, bz) = smoothed_points[j];

                    // Place track foundation at elevation height
                    editor.set_block(IRON_BLOCK, bx, elevation_height, bz, None, None);

                    let prev = if j > 0 {
                        Some(smoothed_points[j - 1])
                    } else {
                        None
                    };
                    let next = if j < smoothed_points.len() - 1 {
                        Some(smoothed_points[j + 1])
                    } else {
                        None
                    };

                    let rail_block = determine_rail_direction(
                        (bx, bz),
                        prev.map(|(x, _, z)| (x, z)),
                        next.map(|(x, _, z)| (x, z)),
                    );

                    // Place rail on top of the foundation
                    editor.set_block(rail_block, bx, elevation_height + 1, bz, None, None);

                    // Place support pillars every pillar_interval blocks
                    if bx % pillar_interval == 0 && bz % pillar_interval == 0 {
                        // Create a pillar from ground level up to the track
                        for y in 1..elevation_height {
                            editor.set_block(IRON_BLOCK, bx, y, bz, None, None);
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{generate_railways, smooth_diagonal_rails};
    use crate::block_definitions::{
        GRAVEL, RAIL_EAST_WEST, RAIL_NORTH_EAST, RAIL_NORTH_SOUTH, RAIL_NORTH_WEST,
        RAIL_SOUTH_EAST, RAIL_SOUTH_WEST, STONE_BRICKS,
    };
    use crate::coordinate_system::{cartesian::XZBBox, geographic::LLBBox};
    use crate::ground::Ground;
    use crate::osm_parser::{ProcessedNode, ProcessedWay};
    use crate::world_editor::WorldEditor;
    use std::collections::HashMap;

    fn build_editor<'a>() -> (tempfile::TempDir, XZBBox, LLBBox, Ground, WorldEditor<'a>) {
        let tmpdir = tempfile::tempdir().unwrap();
        std::fs::create_dir(tmpdir.path().join("region")).unwrap();

        let xzbbox = XZBBox::rect_from_xz_lengths(32.0, 32.0).unwrap();
        let llbbox = LLBBox::new(0.0, 0.0, 1.0, 1.0).unwrap();
        let ground = Ground::new_flat(0);

        let mut editor = WorldEditor::new(tmpdir.path().to_path_buf(), &xzbbox, llbbox);
        editor.set_ground(&ground);

        (tmpdir, xzbbox, llbbox, ground, editor)
    }

    fn sample_railway() -> ProcessedWay {
        let nodes = (0..=5)
            .map(|idx| ProcessedNode {
                id: idx,
                tags: HashMap::new(),
                x: 6 + idx,
                z: 10,
            })
            .collect();

        let mut tags = HashMap::new();
        tags.insert("railway".to_string(), "rail".to_string());
        tags.insert("layer".to_string(), "1".to_string());

        ProcessedWay {
            id: 99,
            nodes,
            tags,
        }
    }

    #[test]
    fn elevated_layer_places_raised_track() {
        let (_tmpdir, _xzbbox, _llbbox, ground, mut editor) = build_editor();
        let way = sample_railway();

        editor.set_ground(&ground);
        generate_railways(&mut editor, &way);

        let rail_blocks = [
            RAIL_NORTH_SOUTH,
            RAIL_EAST_WEST,
            RAIL_NORTH_EAST,
            RAIL_NORTH_WEST,
            RAIL_SOUTH_EAST,
            RAIL_SOUTH_WEST,
        ];

        let midpoint = &way.nodes[3];
        let ground_y = editor.get_absolute_y(midpoint.x, 0, midpoint.z);
        let mut found_rail = None;

        for y in ground_y + 1..ground_y + 20 {
            if editor.check_for_block_absolute(midpoint.x, y, midpoint.z, Some(&rail_blocks), None)
            {
                found_rail = Some(y);
                break;
            }
        }

        let rail_y = found_rail.expect("Rail block not elevated");
        assert!(rail_y > ground_y + 1);

        assert!(editor.check_for_block_absolute(
            midpoint.x,
            rail_y - 1,
            midpoint.z,
            Some(&[GRAVEL]),
            None,
        ));

        let mut support_found = false;
        for node in &way.nodes {
            let node_ground = editor.get_absolute_y(node.x, 0, node.z);
            for y in node_ground + 1..rail_y {
                if editor.check_for_block_absolute(node.x, y, node.z, Some(&[STONE_BRICKS]), None) {
                    support_found = true;
                    break;
                }
            }
            if support_found {
                break;
            }
        }

        assert!(
            support_found,
            "Expected stone support pillar below elevated railway"
        );
    }

    #[test]
    fn test_smooth_diagonal_rails() {
        let points = vec![(0, 0, 0), (1, 0, 1), (2, 0, 2)];
        let smoothed = smooth_diagonal_rails(&points);
        assert!(smoothed.len() > points.len());
    }
}
