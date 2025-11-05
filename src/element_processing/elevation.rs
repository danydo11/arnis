use std::collections::HashMap;

/// Determine the elevation offset for structures that rely on OSM bridge/layer metadata.
///
/// * `tags` - The tag collection of the processed way.
/// * `default_clearance` - Fallback clearance in blocks when only a `bridge=yes` tag is present.
///
/// Returns the number of blocks the structure should be raised above the ground level.
pub fn infer_structure_elevation(tags: &HashMap<String, String>, default_clearance: i32) -> i32 {
    const LAYER_HEIGHT_STEP: i32 = 6;

    let mut offsets: Vec<i32> = Vec::new();

    if let Some(layer) = tags
        .get("layer")
        .and_then(|value| value.parse::<i32>().ok())
    {
        offsets.push((layer).max(0) * LAYER_HEIGHT_STEP);
    }

    if let Some(level) = tags
        .get("level")
        .and_then(|value| value.parse::<i32>().ok())
    {
        offsets.push((level).max(0) * LAYER_HEIGHT_STEP);
    }

    let has_bridge_tag = tags
        .get("bridge")
        .map(|value| value != "no" && !value.is_empty())
        .unwrap_or(false);

    if has_bridge_tag {
        offsets.push(default_clearance.max(3));
    }

    for key in ["bridge:height", "bridge:clearance", "bridge:deck_height"] {
        if let Some(value) = tags.get(key).and_then(parse_height_value) {
            offsets.push(value.max(0));
        }
    }

    offsets
        .into_iter()
        .filter(|offset| *offset > 0)
        .max()
        .or_else(|| has_bridge_tag.then_some(default_clearance.max(3)))
        .unwrap_or(0)
}

fn parse_height_value(value: &str) -> Option<i32> {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return None;
    }

    let mut numeric = String::new();
    for ch in trimmed.chars() {
        if ch.is_ascii_digit() || ch == '.' || (ch == '-' && numeric.is_empty()) {
            numeric.push(ch);
        } else if !numeric.is_empty() {
            break;
        }
    }

    if numeric.is_empty() {
        return None;
    }

    numeric
        .parse::<f32>()
        .ok()
        .map(|value| value.round() as i32)
}

#[cfg(test)]
mod tests {
    use super::infer_structure_elevation;
    use std::collections::HashMap;

    #[test]
    fn prefers_largest_positive_offset() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("layer".to_string(), "2".to_string());
        tags.insert("bridge:clearance".to_string(), "9".to_string());
        assert_eq!(infer_structure_elevation(&tags, 4), 12);
    }

    #[test]
    fn falls_back_to_default_for_plain_bridge() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        assert_eq!(infer_structure_elevation(&tags, 4), 4);
    }

    #[test]
    fn ignores_negative_layers() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("layer".to_string(), "-1".to_string());
        assert_eq!(infer_structure_elevation(&tags, 4), 4);
    }

    #[test]
    fn handles_height_suffixes() {
        let mut tags = HashMap::new();
        tags.insert("bridge:height".to_string(), "5.2m".to_string());
        assert_eq!(infer_structure_elevation(&tags, 3), 5);
    }
}
