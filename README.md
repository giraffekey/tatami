# tatami

Tatami is a roguelike dungeon generation algorithm that creates a multi-floor dungeon layout from a series of randomly oriented, interconnected rectangles.

The library attempts to provide many of the common features found in roguelikes, such as stairs, teleporters, items, enemies and traps. It is intended to be used as a base upon which a fully featured game can be built on.

Note: There is currently a bug where rooms are being generated inside other rooms, as you can see in the example image below. This will be fixed in the future.

## Examples

### Output an image of a generated dungeon

```rust
let dungeon = Dungeon::generate();

dungeon.output_as_image("dungeon.png");
dungeon.output_floor_as_image(0, "floor-1.png");
```

Example output:

![dungeon floor](images/floor-1.png)

### Generate a dungeon for your game

```rust
let dungeon = Dungeon::generate();

for floor in &dungeon.floors {
    for tile in &floor.tiles {
        // Draw tile
    }

    for room in &floor.rooms {
        for item in &room.items {
            match item.rarity {
                1..=20 => // Spawn common item
                21..=40 => // Spawn uncommon item
                41..=60 => // Spawn rare item
                61..=80 => // Spawn epic item
                81..=100 => // Spawn legendary item
                _ => unreachable!(),
            }
        }

        for enemy in &room.enemies {
            match enemy.difficulty {
                1..=20 => // Spawn common enemy
                21..=40 => // Spawn uncommon enemy
                41..=60 => // Spawn rare enemy
                61..=80 => // Spawn epic enemy
                81..=100 => // Spawn legendary enemy
                _ => unreachable!(),
            }
        }

        for trap in &room.traps {
            match trap.difficulty {
                1..=20 => // Spawn common trap
                21..=40 => // Spawn uncommon trap
                41..=60 => // Spawn rare trap
                61..=80 => // Spawn epic trap
                81..=100 => // Spawn legendary trap
                _ => unreachable!(),
            }
        }
    }
}
```
