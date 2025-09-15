//! Tatami is a roguelike dungeon generation algorithm that creates a multi-floor dungeon layout from a series of randomly oriented, interconnected rectangles.
//!
//! The library attempts to provide many of the common features found in roguelikes, such as stairs, teleporters, items, enemies and traps. It is intended to be used as a base upon which a fully featured game can be built on.

use image::{imageops::overlay, open, GenericImageView, ImageError, Rgb, RgbImage};
use itertools::Itertools;
use noise::{NoiseFn, Perlin};
use num_integer::Roots;
use rand::distributions::WeightedIndex;
use rand::prelude::*;
use rand_chacha::ChaCha20Rng;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use serde::{Deserialize, Serialize};
use std::cmp::{self, Ordering};
use std::collections::{BinaryHeap, VecDeque};
use std::path::Path;
use std::sync::{Arc, Mutex};
use strum::IntoEnumIterator;
use strum_macros::EnumIter;

/// The parameters used to generate a dungeon.
///
/// Values can be in units of cells or tiles. Dungeon is first generated on a smaller grid of cells, which are then translated to a larger number of tiles. This is what gives the dungeon its rectangular structure.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct GenerateDungeonParams {
    /// Width and height of dungeon in terms of cells.
    pub dimensions: (u32, u32),
    /// Number of tiles to translate cells to when outputting the final dungeon.
    pub tiles_per_cell: u32,
    /// Number of floors in the dungeon.
    pub num_floors: u32,
    /// Minimum number of main rooms per floor. These rooms are connected by corridors.
    pub min_rooms_per_floor: u32,
    /// Maximum number of main rooms per floor. These rooms are connected by corridors.
    pub max_rooms_per_floor: u32,
    /// Minimum dimensions in cells for main rooms.
    pub min_room_dimensions: (u32, u32),
    /// Maximum dimensions in cells for main rooms.
    pub max_room_dimensions: (u32, u32),
    /// Whether or not main rooms can connect directly with each other. If false, connections require a corridor in between.
    pub adjacent_rooms_allowed: bool,
    /// Minimum dimensions in cells for corridors.
    pub min_corridor_dimensions: (u32, u32),
    /// Maximum dimensions in cells for corridors.
    pub max_corridor_dimensions: (u32, u32),
    /// Dimensions in tiles for walls.
    pub wall_dimensions: (u32, u32),
    /// This value decreases the likelihood of a corridor having one side much longer than the other.
    pub squareness: f32,
    /// Minimum downward stairs per floor. Number of upward stairs will always be equal to the number of downward stairs in the previous floor.
    pub min_stairs_per_floor: u32,
    /// Maximum downward stairs per floor. Number of upward stairs will always be equal to the number of downward stairs in the previous floor.
    pub max_stairs_per_floor: u32,
    /// Minimum pairs of teleporters per floor.
    pub min_teleporters_per_floor: u32,
    /// Maximum pairs of teleporters per floor.
    pub max_teleporters_per_floor: u32,
    /// Minimum number of items per main room or corridor.
    pub min_items_per_room: u32,
    /// Maximum number of items per main room or corridor.
    pub max_items_per_room: u32,
    /// Maximum value for item rarity score. Minimum will always be one. Items tend to be rarer in farther rooms or floors.
    pub max_item_rarity: u32,
    /// Scale to be used for the item density noise map.
    pub item_density_scale: f32,
    /// Scale to be used for the item rarity noise map.
    pub item_rarity_scale: f32,
    /// Minimum number of enemies per main room or corridor.
    pub min_enemies_per_room: u32,
    /// Maximum number of items per main room or corridor.
    pub max_enemies_per_room: u32,
    /// Maximum value for enemy difficulty score. Minimum will always be one. Enemies tend to be more difficult in farther rooms or floors.
    pub max_enemy_difficulty: u32,
    /// Scale to be used for the enemy density noise map.
    pub enemy_density_scale: f32,
    /// Scale to be used for the enemy difficulty noise map.
    pub enemy_difficulty_scale: f32,
    /// Minimum number of traps per main room or corridor.
    pub min_traps_per_room: u32,
    /// Maximum number of enemies per main room or corridor.
    pub max_traps_per_room: u32,
    /// Maximum value for trap difficulty score. Minimum will always be one. Traps tend to be more difficult in farther rooms or floors.
    pub max_trap_difficulty: u32,
    /// Scale to be used for the trap density noise map.
    pub trap_density_scale: f32,
    /// Scale to be used for the trap difficulty noise map.
    pub trap_difficulty_scale: f32,
    /// The difficulty of the farthest room vs the difficulty of the starting room.
    pub difficulty_ratio: f32,
}

impl Default for GenerateDungeonParams {
    fn default() -> Self {
        Self {
            dimensions: (32, 32),
            tiles_per_cell: 6,
            num_floors: 5,
            min_rooms_per_floor: 16,
            max_rooms_per_floor: 16,
            min_room_dimensions: (2, 2),
            max_room_dimensions: (4, 4),
            adjacent_rooms_allowed: false,
            min_corridor_dimensions: (1, 1),
            max_corridor_dimensions: (8, 8),
            wall_dimensions: (2, 2),
            squareness: 1.0,
            min_stairs_per_floor: 3,
            max_stairs_per_floor: 3,
            min_teleporters_per_floor: 3,
            max_teleporters_per_floor: 3,
            min_items_per_room: 1,
            max_items_per_room: 5,
            max_item_rarity: 100,
            item_density_scale: 0.5,
            item_rarity_scale: 2.0,
            min_enemies_per_room: 1,
            max_enemies_per_room: 5,
            max_enemy_difficulty: 100,
            enemy_density_scale: 0.5,
            enemy_difficulty_scale: 2.0,
            min_traps_per_room: 1,
            max_traps_per_room: 5,
            max_trap_difficulty: 100,
            trap_density_scale: 0.5,
            trap_difficulty_scale: 2.0,
            difficulty_ratio: 5.0,
        }
    }
}

// A cell in the mock-up grid
#[derive(Debug, Clone, Copy, PartialEq)]
struct Cell {
    pub i: u32,
    pub j: u32,
}

impl Cell {
    pub fn direction_to(&self, other: &Self) -> Option<Direction> {
        if other.i < self.i && self.i - other.i == 1 && self.j == other.j {
            Some(Direction::Left)
        } else if other.i > self.i && other.i - self.i == 1 && self.j == other.j {
            Some(Direction::Right)
        } else if other.j < self.j && self.j - other.j == 1 && self.i == other.i {
            Some(Direction::Up)
        } else if other.j > self.j && other.j - self.j == 1 && self.i == other.i {
            Some(Direction::Down)
        } else {
            None
        }
    }
}

/// A connection in the mock-up.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct CellConnection {
    /// ID of the connected to room.
    pub id: u32,
    /// Direction the connected to room is in.
    pub direction: Direction,
}

// A room in the mock-up.
#[derive(Debug, Clone)]
struct CellRoom {
    pub id: u32,
    pub kind: RoomKind,
    pub cell: Cell,
    pub width: u32,
    pub height: u32,
    pub connections: Vec<CellConnection>,
}

impl CellRoom {
    pub fn center(&self) -> Cell {
        Cell {
            i: self.cell.i + self.width / 2,
            j: self.cell.j + self.height / 2,
        }
    }

    pub fn cells(&self) -> impl Iterator<Item = Cell> + '_ {
        (0..self.width).flat_map(move |x| {
            (0..self.height).map(move |y| Cell {
                i: self.cell.i + x,
                j: self.cell.j + y,
            })
        })
    }

    pub fn direction_to(&self, other: &Self) -> Option<Direction> {
        self.cells()
            .find_map(|cell| other.cells().find_map(|other| cell.direction_to(&other)))
    }
}

/// X and Y position of a tile.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Deserialize, Serialize)]
pub struct Position {
    pub x: u32,
    pub y: u32,
}

impl Position {
    /// Adjacent positions in cardinal directions.
    pub fn adjacent_4(&self, dimensions: (u32, u32)) -> Vec<Self> {
        let mut positions = Vec::new();

        if self.x > 0 {
            positions.push(Self {
                x: self.x - 1,
                ..*self
            });
        }

        if self.x < dimensions.0 - 1 {
            positions.push(Self {
                x: self.x + 1,
                ..*self
            });
        }

        if self.y > 0 {
            positions.push(Self {
                y: self.y - 1,
                ..*self
            });
        }

        if self.y < dimensions.1 - 1 {
            positions.push(Self {
                y: self.y + 1,
                ..*self
            });
        }

        positions
    }

    /// Adjacent positions in cardinal and diagonal directions.
    pub fn adjacent_8(&self, dimensions: (u32, u32)) -> Vec<Self> {
        let mut positions = Vec::new();

        if self.x > 0 {
            positions.push(Self {
                x: self.x - 1,
                ..*self
            });

            if self.y > 0 {
                positions.push(Self {
                    x: self.x - 1,
                    y: self.y - 1,
                });
            }

            if self.y < dimensions.1 - 1 {
                positions.push(Self {
                    x: self.x - 1,
                    y: self.y + 1,
                });
            }
        }

        if self.x < dimensions.0 - 1 {
            positions.push(Self {
                x: self.x + 1,
                ..*self
            });

            if self.y > 0 {
                positions.push(Self {
                    x: self.x + 1,
                    y: self.y - 1,
                });
            }

            if self.y < dimensions.1 - 1 {
                positions.push(Self {
                    x: self.x + 1,
                    y: self.y + 1,
                });
            }
        }

        if self.y > 0 {
            positions.push(Self {
                y: self.y - 1,
                ..*self
            });
        }

        if self.y < dimensions.1 - 1 {
            positions.push(Self {
                y: self.y + 1,
                ..*self
            });
        }

        positions
    }

    /// Distance between two positions
    pub fn distance(&self, other: Self) -> u32 {
        let dx = self.x as i32 - other.x as i32;
        let dy = self.y as i32 - other.y as i32;
        (dx * dx + dy * dy).sqrt() as u32
    }
}

/// Cardinal directions iterated in clockwise order.
#[derive(
    Debug, Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Deserialize, Serialize, EnumIter,
)]
pub enum Direction {
    Up,
    Right,
    Down,
    Left,
}

impl Direction {
    /// Opposite of current direction.
    pub fn opposite(&self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Left,
            Self::Up => Self::Down,
            Self::Down => Self::Up,
        }
    }
}

/// Connection from one room to another.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct Connection {
    /// ID of the connected to room.
    pub id: u32,
    /// Direction the connected to room is in.
    pub direction: Direction,
    /// Top-left position of the door connecting the room.
    pub door: Position,
}

/// Stairs leading to a different floor.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct Stair {
    /// ID of stairs.
    pub id: u32,
    /// Tile position of stairs.
    pub position: Position,
    /// Leads to next floor if true, leads to previous floor if false.
    pub downwards: bool,
    /// ID of stairs in other floor these stairs are connected to.
    pub connected: u32,
}

/// Teleporter providing instant transportation to another on the same floor.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct Teleporter {
    /// ID of teleporter.
    pub id: u32,
    /// Tile position of teleporter.
    pub position: Position,
    /// ID of teleporter that this teleporter is linked to.
    pub connected: u32,
}

/// An item that can be used to equipped by the player.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct Item {
    /// ID of item instance.
    pub id: u32,
    /// Tile position of item.
    pub position: Position,
    /// Rarity value of item. This value can be used to decide which item is placed at this location.
    pub rarity: u32,
}

/// An enemy that moves toward and attacks the player.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct Enemy {
    /// ID of enemy instance.
    pub id: u32,
    /// Tile position of enemy.
    pub position: Position,
    /// Difficulty value of enemy. This value can be used to decide which enemy is placed at this location.
    pub difficulty: u32,
}

/// A trap that is hidden from the player and creates a negative effect when stepped on.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct Trap {
    /// ID of trap instance.
    pub id: u32,
    /// Tile position of trap.
    pub position: Position,
    /// Difficulty value of trap. This value can be used to decide which trap is placed at this location.
    pub difficulty: u32,
}

/// Type of room.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub enum RoomKind {
    /// Generated first using binary space partitioning.
    Main,
    /// Main rooms are connected by a series of corridors.
    Corridor,
}

/// A traversable room in the dungeon.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Room {
    /// ID of room.
    pub id: u32,
    /// Type of room (Main or Corridor).
    pub kind: RoomKind,
    /// Floor number (indexed by zero) this room is on.
    pub floor: u32,
    /// Position of the top-left tile of the room.
    pub position: Position,
    /// Width (in tiles) of room.
    pub width: u32,
    /// Height (in tiles) of room.
    pub height: u32,
    /// Each room is connected to at least one other room by a door.
    pub connections: Vec<Connection>,
    /// Difficulty value of the room. Based on how far the room is from the starting room.
    pub difficulty: f32,
    /// List of stairs in the room.
    pub stairs: Vec<Stair>,
    /// List of teleporters in the room.
    pub teleporters: Vec<Teleporter>,
    /// List of items in the room.
    pub items: Vec<Item>,
    /// List of enemies in the room.
    pub enemies: Vec<Enemy>,
    /// List of traps in the room.
    pub traps: Vec<Trap>,
    /// Dimensions in tiles for walls.
    pub wall_dimensions: (u32, u32),
}

impl Room {
    /// Center tile of the room (picks top-left if even number of tiles).
    pub fn center(&self) -> Position {
        Position {
            x: self.position.x + self.width / 2,
            y: self.position.y + self.height / 2,
        }
    }

    /// Iterator over all floor tile positions in the room.
    pub fn positions(&self) -> impl Iterator<Item = Position> + '_ {
        let left_wall = (self.wall_dimensions.0 as f32 / 2.0).floor() as u32;
        let right_wall = (self.wall_dimensions.0 as f32 / 2.0).ceil() as u32;
        let top_wall = (self.wall_dimensions.1 as f32 / 2.0).floor() as u32;
        let bottom_wall = (self.wall_dimensions.1 as f32 / 2.0).ceil() as u32;

        (left_wall..self.width - right_wall).flat_map(move |i| {
            (top_wall..self.height - bottom_wall).map(move |j| Position {
                x: self.position.x + i,
                y: self.position.y + j,
            })
        })
    }

    /// Iterator over all empty floor tile positions in the room.
    pub fn empty_positions(&self) -> impl Iterator<Item = Position> + '_ {
        self.positions().filter(|position| {
            !self.stairs.iter().any(|stair| stair.position == *position)
                && !self
                    .teleporters
                    .iter()
                    .any(|teleporter| teleporter.position == *position)
                && !self.items.iter().any(|item| item.position == *position)
                && !self.enemies.iter().any(|enemy| enemy.position == *position)
                && !self.traps.iter().any(|trap| trap.position == *position)
        })
    }

    fn from_cell_room(
        params: &GenerateDungeonParams,
        room: &CellRoom,
        floor: u32,
        rooms: &FxHashMap<u32, CellRoom>,
        kept: &FxHashSet<u32>,
        doors: &FxHashMap<(u32, u32), Position>,
    ) -> Self {
        Self {
            id: room.id,
            kind: room.kind,
            floor,
            position: Position {
                x: room.cell.i * params.tiles_per_cell,
                y: room.cell.j * params.tiles_per_cell,
            },
            width: room.width * params.tiles_per_cell,
            height: room.height * params.tiles_per_cell,
            connections: room
                .connections
                .iter()
                .filter(|connection| {
                    rooms.get(&connection.id).unwrap().kind == RoomKind::Main
                        || kept.contains(&connection.id)
                })
                .map(|connection| Connection {
                    id: connection.id,
                    direction: connection.direction,
                    door: doors[&(room.id, connection.id)],
                })
                .collect(),
            difficulty: 0.0,
            stairs: Vec::new(),
            teleporters: Vec::new(),
            items: Vec::new(),
            enemies: Vec::new(),
            traps: Vec::new(),
            wall_dimensions: params.wall_dimensions,
        }
    }
}

/// A single tile in the dungeon.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub enum Tile {
    /// Traversable.
    Floor,
    /// Non-traversable.
    Wall,
}

/// A floor of the dungeon.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Floor {
    /// The index of this floor, starting at zero for the top floor.
    pub number: u32,
    /// The list of tiles in the floor.
    pub tiles: Vec<Vec<Tile>>,
    /// The list of rooms in the floor.
    pub rooms: Vec<Room>,
}

impl Floor {
    /// Get the tile at the position.
    pub fn tile_at(&self, position: Position) -> Tile {
        self.tiles[position.x as usize][position.y as usize]
    }
}

/// A procedurally generated dungeon, complete with rooms, items and enemies.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Dungeon {
    /// Parameters used to generate the dungeon.
    pub params: GenerateDungeonParams,
    /// A list of floors in the dungeon.
    pub floors: Vec<Floor>,
    /// The ID of the room the player starts in.
    pub starting_room_id: u32,
    /// The initial tile position of the player.
    pub player_position: Position,
}

/// An issue run into during dungeon generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Deserialize, Serialize)]
pub enum TatamiError {
    /// Dungeons must contain at least one floor.
    NoFloors,
    /// Floors must contain at least two rooms.
    NoRooms,
    /// Walls cannot have widths or heights of zero.
    NoWalls,
    /// Rooms cannot have widths or heights of zero.
    InvalidRoomDimensions,
    /// The minimum value for a parameter was set to more than the maximum value.
    MinGreaterThanMax,
    /// Dimensions of dungeon were insufficient for room partition size. Increase dimensions or decrease either max_rooms_per_floor or max_room_dimensions.
    TooSmall,
}

impl Dungeon {
    /// Generate a dungeon with a random seed and default parameters.
    pub fn generate() -> Result<Self, TatamiError> {
        let mut rng = thread_rng();
        let mut seed = [0u8; 32];
        rng.fill(&mut seed);

        Self::generate_with_seed(seed)
    }

    /// Generate a dungeon with default parameters.
    pub fn generate_with_seed(seed: [u8; 32]) -> Result<Self, TatamiError> {
        Self::generate_with_seed_and_params(seed, GenerateDungeonParams::default())
    }

    /// Generate a dungeon with a random seed.
    pub fn generate_with_params(params: GenerateDungeonParams) -> Result<Self, TatamiError> {
        let mut rng = thread_rng();
        let mut seed = [0u8; 32];
        rng.fill(&mut seed);

        Self::generate_with_seed_and_params(seed, params)
    }

    /// Generate a dungeon with the provided seed and parameters.
    pub fn generate_with_seed_and_params(
        seed: [u8; 32],
        params: GenerateDungeonParams,
    ) -> Result<Self, TatamiError> {
        if params.num_floors == 0 {
            return Err(TatamiError::NoFloors);
        }

        if params.min_rooms_per_floor <= 1 || params.max_rooms_per_floor <= 1 {
            return Err(TatamiError::NoRooms);
        }

        if params.wall_dimensions.0 == 0 || params.wall_dimensions.1 == 0 {
            return Err(TatamiError::NoWalls);
        }

        if params.min_room_dimensions.0 == 0 || params.min_room_dimensions.1 == 0 {
            return Err(TatamiError::InvalidRoomDimensions);
        }

        if params.max_room_dimensions.0 == 0 || params.max_room_dimensions.1 == 0 {
            return Err(TatamiError::InvalidRoomDimensions);
        }

        if params.min_corridor_dimensions.0 == 0 || params.min_corridor_dimensions.1 == 0 {
            return Err(TatamiError::InvalidRoomDimensions);
        }

        if params.max_corridor_dimensions.0 == 0 || params.max_corridor_dimensions.1 == 0 {
            return Err(TatamiError::InvalidRoomDimensions);
        }

        if params.min_rooms_per_floor > params.max_rooms_per_floor {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_room_dimensions.0 > params.max_room_dimensions.0 {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_room_dimensions.1 > params.max_room_dimensions.1 {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_corridor_dimensions.0 > params.max_corridor_dimensions.0 {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_corridor_dimensions.1 > params.max_corridor_dimensions.1 {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_stairs_per_floor > params.max_stairs_per_floor {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_teleporters_per_floor > params.max_teleporters_per_floor {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_items_per_room > params.max_items_per_room {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_enemies_per_room > params.max_enemies_per_room {
            return Err(TatamiError::MinGreaterThanMax);
        }

        if params.min_traps_per_room > params.max_traps_per_room {
            return Err(TatamiError::MinGreaterThanMax);
        }

        let (width, height) = params.dimensions;

        let depth = (params.max_rooms_per_floor as f32).log(2.0).ceil() as u32;
        let partitions = 2_u32.pow((depth + 1) / 2);

        let partition_width = if params.adjacent_rooms_allowed {
            width / partitions
        } else {
            (width / partitions).checked_sub(2).unwrap_or(0)
        };
        let partition_height = if params.adjacent_rooms_allowed {
            height / partitions
        } else {
            (height / partitions).checked_sub(2).unwrap_or(0)
        };

        if params.max_room_dimensions.0 > partition_width
            || params.max_room_dimensions.1 > partition_height
        {
            return Err(TatamiError::TooSmall);
        }

        let mut rng = ChaCha20Rng::from_seed(seed);

        let mut floors: Vec<Floor> = Vec::with_capacity(params.num_floors as usize);
        let mut prev_stairs = Vec::new();
        let mut room_id = 0;
        let mut teleporter_id = 0;
        for i in 0..params.num_floors {
            let floor = generate_floor(&mut rng, i, params, &prev_stairs, room_id, teleporter_id);
            prev_stairs = floor
                .rooms
                .iter()
                .flat_map(|room| room.stairs.iter().filter(|stair| stair.downwards).copied())
                .collect();

            if let Some(last_floor) = floors.last_mut() {
                let upward_stairs = floor
                    .rooms
                    .iter()
                    .flat_map(|room| room.stairs.iter().filter(|stair| !stair.downwards).copied());
                for stair in upward_stairs {
                    let last_stair = last_floor
                        .rooms
                        .iter_mut()
                        .find_map(|room| {
                            room.stairs
                                .iter_mut()
                                .find(|other| other.position == stair.position)
                        })
                        .unwrap();
                    last_stair.connected = stair.id;
                }
            }

            room_id += width * height;
            teleporter_id += floor
                .rooms
                .iter()
                .map(|room| room.teleporters.len() as u32)
                .sum::<u32>();

            floors.push(floor);
        }

        let starting_room = floors[0]
            .rooms
            .iter()
            .filter(|room| room.kind == RoomKind::Main)
            .choose(&mut rng)
            .unwrap();
        let starting_room_id = starting_room.id;
        let player_position = starting_room.center();

        let item_density_map = Perlin::new(rng.gen());
        let item_rarity_map = Perlin::new(rng.gen());
        let enemy_density_map = Perlin::new(rng.gen());
        let enemy_difficulty_map = Perlin::new(rng.gen());
        let trap_density_map = Perlin::new(rng.gen());
        let trap_difficulty_map = Perlin::new(rng.gen());

        let rooms = FxHashMap::from_iter(
            floors
                .iter()
                .flat_map(|floor| floor.rooms.clone())
                .map(|room| (room.id, room)),
        );
        let costs = FxHashMap::from_iter(
            rooms
                .par_iter()
                .filter_map(
                    |(id, room)| match pathfind_rooms(starting_room_id, room.id, &rooms) {
                        Some((_, cost)) => Some((*id, cost)),
                        None => None,
                    },
                )
                .collect::<Vec<(u32, u32)>>(),
        );
        let max_cost = costs.values().copied().max().unwrap();
        let max_area = rooms
            .values()
            .map(|room| room.width * room.height)
            .max()
            .unwrap();

        let rng = Arc::new(Mutex::new(rng));
        floors.par_iter_mut().for_each(|floor| {
            floor.rooms.par_iter_mut().for_each(|room| {
                let mut rng = rng.lock().unwrap();
                populate_room(
                    room,
                    &mut *rng,
                    &params,
                    &costs,
                    max_cost,
                    max_area,
                    &item_density_map,
                    &item_rarity_map,
                    &enemy_density_map,
                    &enemy_difficulty_map,
                    &trap_density_map,
                    &trap_difficulty_map,
                );
            })
        });

        Ok(Self {
            params,
            floors,
            starting_room_id,
            player_position,
        })
    }

    /// Output an image representation of all floors in the dungeon.
    pub fn output_as_image<P: AsRef<Path> + Clone + Sync>(
        &self,
        path: P,
        spritesheet: P,
        tile_size: u32,
    ) -> Result<(), ImageError> {
        let player_image = open(spritesheet.clone())
            .unwrap()
            .into_rgb8()
            .view(tile_size * 2, tile_size * 4, tile_size, tile_size)
            .to_image();

        let (width, height) = self.params.dimensions;
        let cols = (self.floors.len() as f32).sqrt().ceil() as u32;
        let rows = (self.floors.len() as f32 / cols as f32).ceil() as u32;
        let width = (width * self.params.tiles_per_cell) as u32;
        let height = (height * self.params.tiles_per_cell) as u32;
        let padding = tile_size * 8;
        let mut img = RgbImage::from_pixel(
            cols * width * tile_size + padding * (cols - 1),
            rows * height * tile_size + padding * (rows - 1),
            Rgb([255, 255, 255]),
        );

        let images: Vec<_> = (0..self.floors.len())
            .collect::<Vec<_>>()
            .par_iter()
            .map(|i| {
                let col = *i as u32 % cols;
                let row = *i as u32 / cols;
                let floor_image = self.floor_to_image(*i, &spritesheet, tile_size);
                (col, row, floor_image)
            })
            .collect();

        for (col, row, floor_image) in images {
            overlay(
                &mut img,
                &floor_image,
                ((col * width) * tile_size) as i64 + (col * padding) as i64,
                ((row * height) * tile_size) as i64 + (row * padding) as i64,
            );
        }

        overlay(
            &mut img,
            &player_image,
            (self.player_position.x * tile_size) as i64,
            (self.player_position.y * tile_size) as i64,
        );

        img.save(path)?;

        Ok(())
    }

    /// Output an image representation of the given floor.
    pub fn output_floor_as_image<P: AsRef<Path>>(
        &self,
        floor_number: u32,
        path: P,
        spritesheet: P,
        tile_size: u32,
    ) -> Result<(), ImageError> {
        let img = self.floor_to_image(floor_number as usize, spritesheet, tile_size);

        img.save(path)?;

        Ok(())
    }

    // Create an image from floor data
    fn floor_to_image<P: AsRef<Path>>(
        &self,
        index: usize,
        spritesheet: P,
        tile_size: u32,
    ) -> RgbImage {
        let floor = &self.floors[index];
        let (width, height) = self.params.dimensions;

        let spritesheet = open(spritesheet).unwrap().into_rgb8();
        let floor_tile_image = spritesheet.view(0, 0, tile_size, tile_size).to_image();
        let wall_tile_image = spritesheet
            .view(tile_size, 0, tile_size, tile_size)
            .to_image();
        let stairs_down_image = spritesheet
            .view(tile_size * 2, 0, tile_size, tile_size)
            .to_image();
        let stairs_up_image = spritesheet
            .view(tile_size * 3, 0, tile_size, tile_size)
            .to_image();
        let teleporter_image = spritesheet
            .view(tile_size * 4, 0, tile_size, tile_size)
            .to_image();
        let common_item_image = spritesheet
            .view(0, tile_size, tile_size, tile_size)
            .to_image();
        let uncommon_item_image = spritesheet
            .view(tile_size, tile_size, tile_size, tile_size)
            .to_image();
        let rare_item_image = spritesheet
            .view(tile_size * 2, tile_size, tile_size, tile_size)
            .to_image();
        let epic_item_image = spritesheet
            .view(tile_size * 3, tile_size, tile_size, tile_size)
            .to_image();
        let legendary_item_image = spritesheet
            .view(tile_size * 4, tile_size, tile_size, tile_size)
            .to_image();
        let common_enemy_image = spritesheet
            .view(0, tile_size * 2, tile_size, tile_size)
            .to_image();
        let uncommon_enemy_image = spritesheet
            .view(tile_size, tile_size * 2, tile_size, tile_size)
            .to_image();
        let rare_enemy_image = spritesheet
            .view(tile_size * 2, tile_size * 2, tile_size, tile_size)
            .to_image();
        let epic_enemy_image = spritesheet
            .view(tile_size * 3, tile_size * 2, tile_size, tile_size)
            .to_image();
        let legendary_enemy_image = spritesheet
            .view(tile_size * 4, tile_size * 2, tile_size, tile_size)
            .to_image();
        let common_trap_image = spritesheet
            .view(0, tile_size * 3, tile_size, tile_size)
            .to_image();
        let uncommon_trap_image = spritesheet
            .view(tile_size, tile_size * 3, tile_size, tile_size)
            .to_image();
        let rare_trap_image = spritesheet
            .view(tile_size * 2, tile_size * 3, tile_size, tile_size)
            .to_image();
        let epic_trap_image = spritesheet
            .view(tile_size * 3, tile_size * 3, tile_size, tile_size)
            .to_image();
        let legendary_trap_image = spritesheet
            .view(tile_size * 4, tile_size * 3, tile_size, tile_size)
            .to_image();
        let door_image = spritesheet
            .view(0, tile_size * 4, tile_size * 2, tile_size * 2)
            .to_image();

        let width = width * self.params.tiles_per_cell;
        let height = height * self.params.tiles_per_cell;
        let mut img = RgbImage::new(width as u32 * tile_size, height as u32 * tile_size);

        for x in 0..width {
            for y in 0..height {
                match floor.tiles[x as usize][y as usize] {
                    Tile::Floor => overlay(
                        &mut img,
                        &floor_tile_image,
                        (x * tile_size) as i64,
                        (y * tile_size) as i64,
                    ),
                    Tile::Wall => overlay(
                        &mut img,
                        &wall_tile_image,
                        (x * tile_size) as i64,
                        (y * tile_size) as i64,
                    ),
                }
            }
        }

        for room in &floor.rooms {
            for connection in &room.connections {
                overlay(
                    &mut img,
                    &door_image,
                    (connection.door.x * tile_size) as i64,
                    (connection.door.y * tile_size) as i64,
                )
            }

            for stair in &room.stairs {
                if stair.downwards {
                    overlay(
                        &mut img,
                        &stairs_down_image,
                        (stair.position.x * tile_size) as i64,
                        (stair.position.y * tile_size) as i64,
                    );
                } else {
                    overlay(
                        &mut img,
                        &stairs_up_image,
                        (stair.position.x * tile_size) as i64,
                        (stair.position.y * tile_size) as i64,
                    );
                }
            }

            for teleporter in &room.teleporters {
                overlay(
                    &mut img,
                    &teleporter_image,
                    (teleporter.position.x * tile_size) as i64,
                    (teleporter.position.y * tile_size) as i64,
                );
            }

            for item in &room.items {
                let common_rarity = self.params.max_item_rarity / 5;
                let uncommon_rarity = self.params.max_item_rarity * 2 / 5;
                let rare_rarity = self.params.max_item_rarity * 3 / 5;
                let epic_rarity = self.params.max_item_rarity * 4 / 5;

                if item.rarity <= common_rarity {
                    overlay(
                        &mut img,
                        &common_item_image,
                        (item.position.x * tile_size) as i64,
                        (item.position.y * tile_size) as i64,
                    );
                } else if item.rarity <= uncommon_rarity {
                    overlay(
                        &mut img,
                        &uncommon_item_image,
                        (item.position.x * tile_size) as i64,
                        (item.position.y * tile_size) as i64,
                    );
                } else if item.rarity <= rare_rarity {
                    overlay(
                        &mut img,
                        &rare_item_image,
                        (item.position.x * tile_size) as i64,
                        (item.position.y * tile_size) as i64,
                    );
                } else if item.rarity <= epic_rarity {
                    overlay(
                        &mut img,
                        &epic_item_image,
                        (item.position.x * tile_size) as i64,
                        (item.position.y * tile_size) as i64,
                    );
                } else {
                    overlay(
                        &mut img,
                        &legendary_item_image,
                        (item.position.x * tile_size) as i64,
                        (item.position.y * tile_size) as i64,
                    );
                }
            }

            for enemy in &room.enemies {
                let common_difficulty = self.params.max_enemy_difficulty / 5;
                let uncommon_difficulty = self.params.max_enemy_difficulty * 2 / 5;
                let rare_difficulty = self.params.max_enemy_difficulty * 3 / 5;
                let epic_difficulty = self.params.max_enemy_difficulty * 4 / 5;

                if enemy.difficulty <= common_difficulty {
                    overlay(
                        &mut img,
                        &common_enemy_image,
                        (enemy.position.x * tile_size) as i64,
                        (enemy.position.y * tile_size) as i64,
                    );
                } else if enemy.difficulty <= uncommon_difficulty {
                    overlay(
                        &mut img,
                        &uncommon_enemy_image,
                        (enemy.position.x * tile_size) as i64,
                        (enemy.position.y * tile_size) as i64,
                    );
                } else if enemy.difficulty <= rare_difficulty {
                    overlay(
                        &mut img,
                        &rare_enemy_image,
                        (enemy.position.x * tile_size) as i64,
                        (enemy.position.y * tile_size) as i64,
                    );
                } else if enemy.difficulty <= epic_difficulty {
                    overlay(
                        &mut img,
                        &epic_enemy_image,
                        (enemy.position.x * tile_size) as i64,
                        (enemy.position.y * tile_size) as i64,
                    );
                } else {
                    overlay(
                        &mut img,
                        &legendary_enemy_image,
                        (enemy.position.x * tile_size) as i64,
                        (enemy.position.y * tile_size) as i64,
                    );
                }
            }

            for trap in &room.traps {
                let common_difficulty = self.params.max_trap_difficulty / 5;
                let uncommon_difficulty = self.params.max_trap_difficulty * 2 / 5;
                let rare_difficulty = self.params.max_trap_difficulty * 3 / 5;
                let epic_difficulty = self.params.max_trap_difficulty * 4 / 5;

                if trap.difficulty <= common_difficulty {
                    overlay(
                        &mut img,
                        &common_trap_image,
                        (trap.position.x * tile_size) as i64,
                        (trap.position.y * tile_size) as i64,
                    );
                } else if trap.difficulty <= uncommon_difficulty {
                    overlay(
                        &mut img,
                        &uncommon_trap_image,
                        (trap.position.x * tile_size) as i64,
                        (trap.position.y * tile_size) as i64,
                    );
                } else if trap.difficulty <= rare_difficulty {
                    overlay(
                        &mut img,
                        &rare_trap_image,
                        (trap.position.x * tile_size) as i64,
                        (trap.position.y * tile_size) as i64,
                    );
                } else if trap.difficulty <= epic_difficulty {
                    overlay(
                        &mut img,
                        &epic_trap_image,
                        (trap.position.x * tile_size) as i64,
                        (trap.position.y * tile_size) as i64,
                    );
                } else {
                    overlay(
                        &mut img,
                        &legendary_trap_image,
                        (trap.position.x * tile_size) as i64,
                        (trap.position.y * tile_size) as i64,
                    );
                }
            }
        }

        img
    }
}

// Generate rooms for a floor in the dungeon, then add stairs and teleporters
fn generate_floor<R: Rng>(
    rng: &mut R,
    floor_number: u32,
    params: GenerateDungeonParams,
    prev_stairs: &[Stair],
    room_id: u32,
    teleporter_id: u32,
) -> Floor {
    let (width, height) = params.dimensions;
    let prev_stair_cells: Box<[Cell]> = prev_stairs
        .iter()
        .map(|stair| Cell {
            i: stair.position.x / params.tiles_per_cell,
            j: stair.position.y / params.tiles_per_cell,
        })
        .collect();

    loop {
        let mut grid = Vec::with_capacity(width as usize);
        for i in 0..width {
            grid.push(Vec::with_capacity(height as usize));
            for _ in 0..height {
                grid[i as usize].push(false);
            }
        }

        let num_rooms = rng.gen_range(params.min_rooms_per_floor..=params.max_rooms_per_floor);
        let mut rooms = generate_main_rooms(
            &mut grid,
            rng,
            width,
            height,
            room_id,
            num_rooms,
            params.min_room_dimensions,
            params.max_room_dimensions,
            params.adjacent_rooms_allowed,
        );
        rooms.extend(generate_corridors(
            &mut grid,
            rng,
            room_id + rooms.len() as u32,
            width,
            height,
            params.min_corridor_dimensions,
            params.max_corridor_dimensions,
            params.squareness,
        ));

        let mut kept = None;
        for _ in 0..2 {
            connect_corridors(rng, &mut rooms);
            if let Some(kept_) = connect_rooms(rng, &rooms, &prev_stair_cells) {
                kept = Some(kept_);
                break;
            }
        }

        if let Some(kept) = kept {
            let (tiles, doors) = generate_tiles(
                &rooms,
                &kept,
                width,
                height,
                params.tiles_per_cell,
                params.wall_dimensions,
            );

            let mut rooms: Vec<_> = rooms
                .values()
                .filter(|room| room.kind == RoomKind::Main || kept.contains(&room.id))
                .map(|room| {
                    Room::from_cell_room(&params, room, floor_number, &rooms, &kept, &doors)
                })
                .collect();

            let last_stair_id = prev_stairs.iter().map(|stair| stair.id).max().unwrap_or(0);
            for (i, stair) in prev_stairs.iter().enumerate() {
                for room in &mut rooms {
                    if room.positions().contains(&stair.position) {
                        room.stairs.push(Stair {
                            id: last_stair_id + i as u32,
                            position: stair.position,
                            downwards: false,
                            connected: stair.id,
                        });
                        break;
                    }
                }
            }
            let last_stair_id = last_stair_id + prev_stairs.len() as u32;

            let num_stairs =
                rng.gen_range(params.min_stairs_per_floor..=params.max_stairs_per_floor);
            for i in 0..num_stairs {
                let room = rooms.choose_mut(rng).unwrap();
                let position = room.empty_positions().choose(rng).unwrap();
                room.stairs.push(Stair {
                    id: last_stair_id + i,
                    position,
                    downwards: true,
                    connected: 0,
                });
            }

            let num_teleporters =
                rng.gen_range(params.min_teleporters_per_floor..=params.max_teleporters_per_floor);
            for i in 0..num_teleporters {
                let id1 = teleporter_id + i * 2;
                let id2 = teleporter_id + i * 2 + 1;

                for (id, connected) in [(id1, id2), (id2, id1)] {
                    let room = rooms.choose_mut(rng).unwrap();
                    let position = room.empty_positions().choose(rng).unwrap();
                    room.teleporters.push(Teleporter {
                        id,
                        position,
                        connected,
                    });
                }
            }

            return Floor {
                number: floor_number,
                tiles,
                rooms,
            };
        }
    }
}

// Populate the room with items, enemies and traps based on noise maps and distance from starting room
fn populate_room<R: Rng>(
    room: &mut Room,
    rng: &mut R,
    params: &GenerateDungeonParams,
    costs: &FxHashMap<u32, u32>,
    max_cost: u32,
    max_area: u32,
    item_density_map: &Perlin,
    item_rarity_map: &Perlin,
    enemy_density_map: &Perlin,
    enemy_difficulty_map: &Perlin,
    trap_density_map: &Perlin,
    trap_difficulty_map: &Perlin,
) {
    let (width, height) = params.dimensions;

    let cost = match costs.get(&room.id) {
        Some(cost) => *cost,
        None => return,
    };
    room.difficulty = 1.0 / params.difficulty_ratio
        + cost as f32 / max_cost as f32 * (1.0 - 1.0 / params.difficulty_ratio);

    let density_ratio = (room.width * room.height) as f32 / max_area as f32;

    let min_items = params.min_items_per_room;
    let max_items =
        min_items + ((params.max_items_per_room - min_items) as f32 * density_ratio) as u32;
    let item_density_noise = get_room_noise(room, item_density_map, width, height);
    let num_items = if min_items == max_items {
        max_items
    } else {
        weighted_random(
            rng,
            min_items,
            max_items,
            item_density_noise,
            params.item_density_scale,
        )
    };

    for _ in 0..num_items {
        let position = room.empty_positions().choose(rng).unwrap();

        let item_rarity_noise = get_room_noise(room, item_rarity_map, width, height);
        let rarity = weighted_random(
            rng,
            1,
            params.max_item_rarity,
            item_rarity_noise,
            params.item_rarity_scale * (1.0 - room.difficulty),
        );

        room.items.push(Item {
            id: rng.gen(),
            position,
            rarity,
        });
    }

    let min_enemies = params.min_enemies_per_room;
    let max_enemies =
        min_enemies + ((params.max_enemies_per_room - min_enemies) as f32 * density_ratio) as u32;
    let enemy_density_noise = get_room_noise(room, enemy_density_map, width, height);
    let num_enemies = if min_enemies == max_enemies {
        max_enemies
    } else {
        weighted_random(
            rng,
            min_enemies,
            max_enemies,
            enemy_density_noise,
            params.enemy_density_scale,
        )
    };

    for _ in 0..num_enemies {
        let position = room.empty_positions().choose(rng).unwrap();

        let enemy_difficulty_noise = get_room_noise(room, &enemy_difficulty_map, width, height);
        let difficulty = weighted_random(
            rng,
            1,
            params.max_enemy_difficulty,
            enemy_difficulty_noise,
            params.enemy_difficulty_scale * (1.0 - room.difficulty),
        );

        room.enemies.push(Enemy {
            id: rng.gen(),
            position,
            difficulty,
        });
    }

    let min_traps = params.min_traps_per_room;
    let max_traps =
        min_traps + ((params.max_traps_per_room - min_traps) as f32 * density_ratio) as u32;
    let trap_density_noise = get_room_noise(room, trap_density_map, width, height);
    let num_traps = if min_traps == max_traps {
        max_traps
    } else {
        weighted_random(
            rng,
            min_traps,
            max_traps,
            trap_density_noise,
            params.trap_density_scale,
        )
    };

    for _ in 0..num_traps {
        let position = room.empty_positions().choose(rng).unwrap();

        let trap_difficulty_noise = get_room_noise(room, trap_difficulty_map, width, height);
        let difficulty = weighted_random(
            rng,
            1,
            params.max_trap_difficulty,
            trap_difficulty_noise,
            params.trap_difficulty_scale * (1.0 - room.difficulty),
        );

        room.traps.push(Trap {
            id: rng.gen(),
            position,
            difficulty,
        });
    }
}

// Generate zero to one main rooms for each partition
fn generate_main_rooms<R: Rng>(
    grid: &mut [Vec<bool>],
    rng: &mut R,
    width: u32,
    height: u32,
    room_id: u32,
    num_rooms: u32,
    min_room_dimensions: (u32, u32),
    max_room_dimensions: (u32, u32),
    adjacent_rooms_allowed: bool,
) -> FxHashMap<u32, CellRoom> {
    let split_horizontally = rng.gen();
    let depth = (num_rooms as f32).log(2.0).ceil() as u32;
    let mut partitions = create_partitions(0, 0, width, height, split_horizontally, depth);
    partitions.shuffle(rng);

    let rooms = FxHashMap::from_iter(partitions.iter().take(num_rooms as usize).enumerate().map(
        |(index, (i, j, width, height))| {
            let id = room_id + index as u32;
            let room_width = rng.gen_range(min_room_dimensions.0..=max_room_dimensions.0);
            let room_height = rng.gen_range(min_room_dimensions.1..=max_room_dimensions.1);

            let (i, j) = if adjacent_rooms_allowed {
                let i = rng.gen_range(*i..=i + width - room_width);
                let j = rng.gen_range(*j..=j + height - room_height);
                (i, j)
            } else {
                let i = rng.gen_range(i + 1..=i + width - 1 - room_width);
                let j = rng.gen_range(j + 1..=j + height - 1 - room_height);
                (i, j)
            };

            let room = CellRoom {
                id,
                kind: RoomKind::Main,
                cell: Cell { i, j },
                width: room_width,
                height: room_height,
                connections: Vec::new(),
            };
            (id, room)
        },
    ));

    for room in rooms.values() {
        for x in 0..room.width {
            for y in 0..room.height {
                let i = (room.cell.i + x) as usize;
                let j = (room.cell.j + y) as usize;
                grid[i][j] = true;
            }
        }
    }

    rooms
}

// Use binary space partitioning to recursively split the grid into rectangles of equal dimensions
fn create_partitions(
    i: u32,
    j: u32,
    width: u32,
    height: u32,
    split_horizontally: bool,
    depth: u32,
) -> Vec<(u32, u32, u32, u32)> {
    if depth == 0 {
        return vec![(i, j, width, height)];
    }

    if split_horizontally {
        [
            // Left
            create_partitions(i, j, width / 2, height, false, depth - 1),
            // Right
            create_partitions(i + width / 2, j, width / 2, height, false, depth - 1),
        ]
        .concat()
    } else {
        [
            // Top
            create_partitions(i, j, width, height / 2, true, depth - 1),
            // Bottom
            create_partitions(i, j + height / 2, width, height / 2, true, depth - 1),
        ]
        .concat()
    }
}

// Fill the mock-up grid with random rectangles
fn generate_corridors<R: Rng>(
    grid: &mut [Vec<bool>],
    rng: &mut R,
    room_id: u32,
    width: u32,
    height: u32,
    min_corridor_dimensions: (u32, u32),
    max_corridor_dimensions: (u32, u32),
    squareness: f32,
) -> FxHashMap<u32, CellRoom> {
    let mut corridors = Vec::new();
    let mut iter = 0;
    loop {
        // Every 10 iterations, add new 1x1 squares to random positions in the grid
        if iter % 10 == 0 {
            let mut empty_cells: Vec<_> = cells(width, height)
                .filter(|cell| !grid[cell.i as usize][cell.j as usize])
                .collect();

            if empty_cells.is_empty() {
                return FxHashMap::default();
            }

            empty_cells.shuffle(rng);

            let empty_cells = &empty_cells[0..cmp::max(empty_cells.len() / 8, 1)];

            for cell in empty_cells {
                grid[cell.i as usize][cell.j as usize] = true;
            }

            let len = corridors.len() as u32;
            corridors.extend(empty_cells.iter().enumerate().map(|(i, cell)| CellRoom {
                id: room_id + len + i as u32,
                kind: RoomKind::Corridor,
                cell: *cell,
                width: 1,
                height: 1,
                connections: Vec::new(),
            }))
        }

        // For each corridor, pick a random direction and grow in it if all adjacent spaces are available
        corridors.shuffle(rng);
        corridors.iter_mut().for_each(|corridor| {
            let weights = if corridor.width > corridor.height {
                let n = (corridor.width - corridor.height) as f32 * squareness;
                [1.0 + n, 1.0, 1.0 + n, 1.0]
            } else if corridor.width < corridor.height {
                let n = (corridor.height - corridor.width) as f32 * squareness;
                [1.0, 1.0 + n, 1.0, 1.0 + n]
            } else {
                [1.0, 1.0, 1.0, 1.0]
            };
            let dist = WeightedIndex::new(&weights).unwrap();
            let direction = Direction::iter().nth(dist.sample(rng)).unwrap();

            match direction {
                Direction::Left => {
                    if corridor.cell.i == 0 || corridor.width >= max_corridor_dimensions.0 {
                        return;
                    }
                    if (0..corridor.height)
                        .map(|y| Cell {
                            i: corridor.cell.i - 1,
                            j: corridor.cell.j + y,
                        })
                        .any(|cell| grid[cell.i as usize][cell.j as usize])
                    {
                        return;
                    }
                }
                Direction::Right => {
                    if corridor.cell.i + corridor.width >= width
                        || corridor.width >= max_corridor_dimensions.0
                    {
                        return;
                    }
                    if (0..corridor.height)
                        .map(|y| Cell {
                            i: corridor.cell.i + corridor.width,
                            j: corridor.cell.j + y,
                        })
                        .any(|cell| grid[cell.i as usize][cell.j as usize])
                    {
                        return;
                    }
                }
                Direction::Up => {
                    if corridor.cell.j == 0 || corridor.height >= max_corridor_dimensions.1 {
                        return;
                    }
                    if (0..corridor.width)
                        .map(|x| Cell {
                            i: corridor.cell.i + x,
                            j: corridor.cell.j - 1,
                        })
                        .any(|cell| grid[cell.i as usize][cell.j as usize])
                    {
                        return;
                    }
                }
                Direction::Down => {
                    if corridor.cell.j + corridor.height >= height
                        || corridor.height >= max_corridor_dimensions.1
                    {
                        return;
                    }
                    if (0..corridor.width)
                        .map(|x| Cell {
                            i: corridor.cell.i + x,
                            j: corridor.cell.j + corridor.height,
                        })
                        .any(|cell| grid[cell.i as usize][cell.j as usize])
                    {
                        return;
                    }
                }
            };

            match direction {
                Direction::Left => {
                    corridor.cell.i -= 1;
                    corridor.width += 1;
                }
                Direction::Right => {
                    corridor.width += 1;
                }
                Direction::Up => {
                    corridor.cell.j -= 1;
                    corridor.height += 1;
                }
                Direction::Down => {
                    corridor.height += 1;
                }
            };

            corridor.cells().for_each(|cell| {
                grid[cell.i as usize][cell.j as usize] = true;
            });
        });

        // If all cells in grid are filled, break out of the loop
        if cells(width, height).all(|cell| grid[cell.i as usize][cell.j as usize]) {
            break;
        }

        iter += 1;
    }

    FxHashMap::from_iter(
        corridors
            .iter()
            .filter(|room| {
                room.width >= min_corridor_dimensions.0 && room.height >= min_corridor_dimensions.1
            })
            .map(|room| (room.id, room.clone())),
    )
}

// Creates connections between all main rooms and corridors
fn connect_corridors<R: Rng>(rng: &mut R, rooms: &mut FxHashMap<u32, CellRoom>) {
    let mut to_connect: Vec<_> = rooms.keys().copied().collect();
    to_connect.shuffle(rng);

    for id in to_connect {
        let room = rooms.get(&id).unwrap();
        if room.connections.len() >= 3 {
            continue;
        }

        let area = room.height * room.width;
        let chance = (area - 1) as f32 / (area * 4) as f32;
        let num_connections = if room.connections.len() == 2 {
            1
        } else if rng.gen::<f32>() < chance {
            2
        } else {
            1
        };

        let room_connections = room.connections.clone();
        let mut connections = neighbors(&room, &rooms)
            .filter(|connection| {
                let connected = rooms.get(&connection.id).unwrap();
                !room_connections
                    .iter()
                    .any(|other| connection.id == other.id)
                    && connected.connections.len() < 3
            })
            .collect::<Vec<_>>();
        connections.shuffle(rng);

        for connection in connections.iter().take(num_connections) {
            let connected = rooms.get_mut(&connection.id).unwrap();
            connected.connections.push(CellConnection {
                id,
                direction: connection.direction.opposite(),
            });

            let room = rooms.get_mut(&id).unwrap();
            room.connections.push(*connection);
        }
    }
}

// Each main room pathfinds to another main room through corridors based on their connections. Unused corridors are deleted.
fn connect_rooms<R: Rng>(
    rng: &mut R,
    rooms: &FxHashMap<u32, CellRoom>,
    prev_stair_cells: &[Cell],
) -> Option<FxHashSet<u32>> {
    let main_rooms: FxHashMap<u32, CellRoom> = FxHashMap::from_iter(
        rooms
            .iter()
            .filter(|(_, room)| room.kind == RoomKind::Main)
            .map(|(id, room)| (*id, room.clone())),
    );

    let start = *main_rooms.keys().choose(rng).unwrap();
    let mut queue = VecDeque::new();
    let mut visited = FxHashSet::default();
    visited.reserve(main_rooms.len());
    queue.push_back(start);
    visited.insert(start);

    let mut kept: FxHashSet<u32> = FxHashSet::default();
    let mut paths = Vec::with_capacity(main_rooms.len() - 1);
    while let Some(id) = queue.pop_front() {
        let room = main_rooms.get(&id).unwrap();

        let other = main_rooms
            .values()
            .filter_map(|other| {
                if visited.contains(&other.id) {
                    None
                } else {
                    Some((other, cell_room_distance(room, other)))
                }
            })
            .sorted_by(|(_, a_dist), (_, b_dist)| a_dist.cmp(&b_dist))
            .take(3)
            .choose(rng);

        if let Some((other, _)) = other {
            if let Some((path, _)) = pathfind_cell_rooms(room.id, other.id, &rooms) {
                let path = &path[1..path.len() - 1];
                kept.extend(path);
                paths.push(path.to_vec());
            } else {
                return None;
            }

            queue.push_back(other.id);
            visited.insert(other.id);
        }
    }

    // Ensure that rooms for upward stairs are included in the floor.
    let stair_rooms: Vec<_> = rooms
        .values()
        .filter(|room| {
            room.kind == RoomKind::Corridor
                && !kept.contains(&room.id)
                && room.cells().any(|cell| prev_stair_cells.contains(&cell))
        })
        .collect();
    for room in stair_rooms {
        let goal = if rng.gen() {
            *main_rooms.keys().choose(rng).unwrap()
        } else {
            *kept.iter().choose(rng).unwrap()
        };
        if let Some((path, _)) = pathfind_cell_rooms(room.id, goal, &rooms) {
            kept.insert(room.id);
            if path.len() >= 2 {
                let path = &path[1..path.len() - 1];
                kept.extend(path);
            }
        } else {
            return None;
        }
    }

    Some(kept)
}

// Generate a grid of tiles for the floor based on rooms and doors
fn generate_tiles(
    rooms: &FxHashMap<u32, CellRoom>,
    kept: &FxHashSet<u32>,
    width: u32,
    height: u32,
    tiles_per_cell: u32,
    wall_dimensions: (u32, u32),
) -> (Vec<Vec<Tile>>, FxHashMap<(u32, u32), Position>) {
    let mut tiles = Vec::with_capacity((width * tiles_per_cell) as usize);
    for x in 0..width * tiles_per_cell {
        tiles.push(Vec::with_capacity((height * tiles_per_cell) as usize));
        for _ in 0..height * tiles_per_cell {
            tiles[x as usize].push(Tile::Wall);
        }
    }

    let mut doors = FxHashMap::default();
    for room in rooms.values() {
        if room.kind == RoomKind::Corridor && !kept.contains(&room.id) {
            continue;
        }

        let left_wall = (wall_dimensions.0 as f32 / 2.0).floor() as u32;
        let right_wall = (wall_dimensions.0 as f32 / 2.0).ceil() as u32;
        let top_wall = (wall_dimensions.1 as f32 / 2.0).floor() as u32;
        let bottom_wall = (wall_dimensions.1 as f32 / 2.0).ceil() as u32;

        let i = room.cell.i * tiles_per_cell;
        let j = room.cell.j * tiles_per_cell;
        let width = room.width * tiles_per_cell;
        let height = room.height * tiles_per_cell;

        for x in left_wall..width - right_wall {
            for y in top_wall..height - bottom_wall {
                tiles[(i + x) as usize][(j + y) as usize] = Tile::Floor;
            }
        }

        for connection in &room.connections {
            let connected = rooms.get(&connection.id).unwrap();
            if connected.kind == RoomKind::Corridor && !kept.contains(&connection.id) {
                continue;
            }

            let shared_cells = room.cells().filter(|cell| {
                connected
                    .cells()
                    .any(|other| cell.direction_to(&other).is_some())
            });

            let (door, door_tiles) = match connection.direction {
                Direction::Left => {
                    let js: Vec<_> = shared_cells.map(|cell| cell.j).collect();
                    let min_j = js.iter().min().unwrap();
                    let max_j = js.iter().max().unwrap();

                    let x = i;
                    let y = (min_j * tiles_per_cell + (max_j + 1) * tiles_per_cell - 1) / 2;
                    let door_tiles: Vec<_> = (0..left_wall)
                        .flat_map(|i| [Position { x: x + i, y }, Position { x: x + i, y: y + 1 }])
                        .collect();
                    (Position { x: x - 1, y }, door_tiles)
                }
                Direction::Right => {
                    let js: Vec<_> = shared_cells.map(|cell| cell.j).collect();
                    let min_j = js.iter().min().unwrap();
                    let max_j = js.iter().max().unwrap();

                    let x = i + width - right_wall;
                    let y = (min_j * tiles_per_cell + (max_j + 1) * tiles_per_cell - 1) / 2;
                    let door_tiles: Vec<_> = (0..right_wall)
                        .flat_map(|i| [Position { x: x + i, y }, Position { x: x + i, y: y + 1 }])
                        .collect();
                    (
                        Position {
                            x: x + right_wall - 1,
                            y,
                        },
                        door_tiles,
                    )
                }
                Direction::Up => {
                    let is: Vec<_> = shared_cells.map(|cell| cell.i).collect();
                    let min_i = is.iter().min().unwrap();
                    let max_i = is.iter().max().unwrap();

                    let x = (min_i * tiles_per_cell + (max_i + 1) * tiles_per_cell - 1) / 2;
                    let y = j;
                    let door_tiles: Vec<_> = (0..top_wall)
                        .flat_map(|j| [Position { x, y: y + j }, Position { x: x + 1, y: y + j }])
                        .collect();
                    (Position { x, y: y - 1 }, door_tiles)
                }
                Direction::Down => {
                    let is: Vec<_> = shared_cells.map(|cell| cell.i).collect();
                    let min_i = is.iter().min().unwrap();
                    let max_i = is.iter().max().unwrap();

                    let x = (min_i * tiles_per_cell + (max_i + 1) * tiles_per_cell - 1) / 2;
                    let y = j + height - bottom_wall;
                    let door_tiles: Vec<_> = (0..bottom_wall)
                        .flat_map(|j| [Position { x, y: y + j }, Position { x: x + 1, y: y + j }])
                        .collect();
                    (
                        Position {
                            x,
                            y: y + bottom_wall - 1,
                        },
                        door_tiles,
                    )
                }
            };

            for position in &door_tiles {
                tiles[position.x as usize][position.y as usize] = Tile::Floor;
            }

            doors.insert((room.id, connection.id), door);
        }
    }

    (tiles, doors)
}

// An iterator over all cells in a floor
fn cells(width: u32, height: u32) -> impl Iterator<Item = Cell> {
    (0..width).flat_map(move |i| (0..height).map(move |j| Cell { i, j }))
}

// An iterator over all adjacent rooms
fn neighbors(
    room: &CellRoom,
    rooms: &FxHashMap<u32, CellRoom>,
) -> impl Iterator<Item = CellConnection> {
    rooms
        .values()
        .filter_map(|other| match room.direction_to(other) {
            Some(direction) if room.id != other.id => Some(CellConnection {
                id: other.id,
                direction,
            }),
            _ => None,
        })
        .sorted_by(|a, b| a.direction.cmp(&b.direction))
}

// Average perlin noise for a room
fn get_room_noise(room: &Room, perlin: &Perlin, width: u32, height: u32) -> f32 {
    let noise_sum: f32 = (0..room.width)
        .flat_map(|i| {
            (0..room.height).map(move |j| {
                perlin.get([
                    (room.floor * width) as f64 + (room.position.x + i) as f64 * 0.01,
                    (room.floor * height) as f64 + (room.position.y + j) as f64 * 0.01,
                ]) as f32
            })
        })
        .sum();
    let area = (room.width * room.height) as f32;
    noise_sum / area
}

// Pick a random value from a range of integers weighted by a noise value
// Larger values are less likely to be picked than smaller ones
fn weighted_random<R: Rng>(rng: &mut R, min: u32, max: u32, noise: f32, scale: f32) -> u32 {
    let values: Vec<_> = (min..=max).collect();
    let noise = (noise + 1.0) / 2.0;
    let weights: Vec<_> = (0..values.len())
        .map(|i| ((i + 1) as f32).powf(1.0 + scale * noise))
        .rev()
        .collect();
    let dist = WeightedIndex::new(weights).unwrap();
    values[dist.sample(rng)]
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Frontier {
    priority: u32,
    id: u32,
}

impl Ord for Frontier {
    fn cmp(&self, other: &Self) -> Ordering {
        other.priority.cmp(&self.priority)
    }
}

impl PartialOrd for Frontier {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Pathfind between mock-up rooms based on their connections
fn pathfind_cell_rooms(
    start: u32,
    goal: u32,
    rooms: &FxHashMap<u32, CellRoom>,
) -> Option<(Vec<u32>, u32)> {
    let mut frontier = BinaryHeap::new();
    let mut came_from = FxHashMap::default();
    let mut costs = FxHashMap::default();

    frontier.push(Frontier {
        priority: 0,
        id: start,
    });
    costs.insert(start, 0);

    let goal_room = rooms.get(&goal).unwrap();
    while let Some(Frontier { priority: _, id }) = frontier.pop() {
        if id == goal {
            break;
        }

        let room = rooms.get(&id).unwrap();
        for connection in &room.connections {
            let new_cost = costs.get(&id).unwrap() + 1;
            if !costs.contains_key(&connection.id) || new_cost < *costs.get(&connection.id).unwrap()
            {
                let connected_room = rooms.get(&connection.id).unwrap();
                frontier.push(Frontier {
                    priority: new_cost + cell_room_distance(connected_room, goal_room),
                    id: connection.id,
                });
                came_from.insert(connection.id, Some(id));
                costs.insert(connection.id, new_cost);
            }
        }
    }

    let mut id = goal;
    let mut path = Vec::new();

    while id != start {
        path.push(id);
        id = match came_from.get(&id) {
            Some(Some(id)) => *id,
            _ => return None,
        };
    }
    path.push(start);
    path.reverse();

    let cost = costs.get(&goal).unwrap();

    Some((path, *cost))
}

// Distance between the center of two mock-up rooms
fn cell_room_distance(a: &CellRoom, b: &CellRoom) -> u32 {
    let a = a.center();
    let b = b.center();
    let (i_diff, j_diff) = (a.i as f32 - b.i as f32, a.j as f32 - b.j as f32);
    (i_diff * i_diff + j_diff * j_diff).sqrt() as u32
}

// Pathfind between floor rooms based on their connections, stairs and teleporters
fn pathfind_rooms(start: u32, goal: u32, rooms: &FxHashMap<u32, Room>) -> Option<(Vec<u32>, u32)> {
    let mut frontier = BinaryHeap::new();
    let mut came_from = FxHashMap::default();
    let mut costs = FxHashMap::default();

    frontier.push(Frontier {
        priority: 0,
        id: start,
    });
    costs.insert(start, 0);

    let goal_room = rooms.get(&goal).unwrap();
    while let Some(Frontier { priority: _, id }) = frontier.pop() {
        if id == goal {
            break;
        }

        let room = rooms.get(&id).unwrap();

        // Doors have a cost of 1, stairs have a cost of 5, teleporters have a cost of 2
        let connections = room
            .connections
            .iter()
            .map(|connection| (connection.id, 1))
            .chain(room.stairs.iter().filter_map(|stair| {
                rooms.values().find_map(|room| {
                    room.stairs.iter().find_map(|other| {
                        if stair.connected == other.id {
                            Some((room.id, 5))
                        } else {
                            None
                        }
                    })
                })
            }))
            .chain(room.teleporters.iter().filter_map(|teleporter| {
                rooms.values().find_map(|room| {
                    room.teleporters.iter().find_map(|other| {
                        if teleporter.connected == other.id {
                            Some((room.id, 2))
                        } else {
                            None
                        }
                    })
                })
            }));

        for (connection_id, cost) in connections {
            let new_cost = costs.get(&id).unwrap() + cost;
            if !costs.contains_key(&connection_id) || new_cost < *costs.get(&connection_id).unwrap()
            {
                let connected_room = rooms.get(&connection_id).unwrap();
                frontier.push(Frontier {
                    priority: new_cost + room_distance(connected_room, goal_room),
                    id: connection_id,
                });
                came_from.insert(connection_id, Some(id));
                costs.insert(connection_id, new_cost);
            }
        }
    }

    let mut id = goal;
    let mut path = Vec::new();

    while id != start {
        path.push(id);
        id = match came_from.get(&id) {
            Some(Some(id)) => *id,
            _ => return None,
        };
    }
    path.push(start);
    path.reverse();

    let cost = costs.get(&goal).unwrap();

    Some((path, *cost))
}

// Distance between the center of two floor rooms
fn room_distance(a: &Room, b: &Room) -> u32 {
    let a = a.center();
    let b = b.center();
    let (x_diff, y_diff) = (a.x as f32 - b.x as f32, a.y as f32 - b.y as f32);
    (x_diff * x_diff + y_diff * y_diff).sqrt() as u32
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    // Use `cargo test --release -- --nocapture`
    #[test]
    fn bench_dungeon_generation() {
        let mut elapsed = Duration::from_secs(0);
        for _ in 0..100 {
            let now = Instant::now();
            assert!(Dungeon::generate().is_ok());
            elapsed += now.elapsed();
        }
        println!("{:?}", elapsed / 100);
    }

    #[test]
    fn output_dungeon_as_image() {
        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            num_floors: 9,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_ok());
        let dungeon = res.unwrap();

        let res = dungeon.output_as_image("images/dungeon.png", "images/spritesheet.png", 8);
        assert!(res.is_ok());
    }

    #[test]
    fn output_floor_as_image() {
        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (16, 16),
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (2, 2),
            adjacent_rooms_allowed: true,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_ok());
        let dungeon = res.unwrap();

        let res =
            dungeon.output_floor_as_image(0, "images/floor-1.png", "images/spritesheet.png", 8);
        assert!(res.is_ok());
    }

    #[test]
    fn error_no_floors() {
        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            num_floors: 0,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::NoFloors);
    }

    #[test]
    fn error_no_rooms() {
        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_rooms_per_floor: 0,
            max_rooms_per_floor: 0,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::NoRooms);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_rooms_per_floor: 0,
            max_rooms_per_floor: 1,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::NoRooms);
    }

    #[test]
    fn error_no_walls() {
        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            wall_dimensions: (0, 0),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::NoWalls);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            wall_dimensions: (1, 0),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::NoWalls);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            wall_dimensions: (0, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::NoWalls);
    }

    #[test]
    fn error_invalid_room_dimensions() {
        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_room_dimensions: (0, 0),
            max_room_dimensions: (0, 0),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::InvalidRoomDimensions);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_room_dimensions: (0, 0),
            max_room_dimensions: (1, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::InvalidRoomDimensions);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_room_dimensions: (1, 0),
            max_room_dimensions: (1, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::InvalidRoomDimensions);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_room_dimensions: (0, 1),
            max_room_dimensions: (1, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::InvalidRoomDimensions);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_corridor_dimensions: (0, 0),
            max_corridor_dimensions: (0, 0),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::InvalidRoomDimensions);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_corridor_dimensions: (0, 0),
            max_corridor_dimensions: (1, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::InvalidRoomDimensions);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_corridor_dimensions: (1, 0),
            max_corridor_dimensions: (1, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::InvalidRoomDimensions);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_corridor_dimensions: (0, 1),
            max_corridor_dimensions: (1, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::InvalidRoomDimensions);
    }

    #[test]
    fn error_min_greater_than_max() {
        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_rooms_per_floor: 3,
            max_rooms_per_floor: 2,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_room_dimensions: (2, 2),
            max_room_dimensions: (1, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_room_dimensions: (1, 2),
            max_room_dimensions: (2, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_room_dimensions: (2, 1),
            max_room_dimensions: (1, 2),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_corridor_dimensions: (2, 2),
            max_corridor_dimensions: (1, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_corridor_dimensions: (1, 2),
            max_corridor_dimensions: (2, 1),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_corridor_dimensions: (2, 1),
            max_corridor_dimensions: (1, 2),
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_stairs_per_floor: 2,
            max_stairs_per_floor: 1,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_teleporters_per_floor: 2,
            max_teleporters_per_floor: 1,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_items_per_room: 2,
            max_items_per_room: 1,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_enemies_per_room: 2,
            max_enemies_per_room: 1,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            min_traps_per_room: 2,
            max_traps_per_room: 1,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::MinGreaterThanMax);
    }

    #[test]
    fn error_too_small() {
        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (2, 2),
            num_floors: 1,
            min_rooms_per_floor: 2,
            max_rooms_per_floor: 2,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (1, 1),
            adjacent_rooms_allowed: true,
            min_stairs_per_floor: 0,
            max_stairs_per_floor: 0,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_ok());

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (3, 3),
            adjacent_rooms_allowed: false,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_ok());

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (5, 5),
            adjacent_rooms_allowed: true,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_ok());

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (16, 16),
            num_floors: 1,
            min_rooms_per_floor: 8,
            max_rooms_per_floor: 8,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (2, 2),
            adjacent_rooms_allowed: false,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_ok());

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (16, 16),
            num_floors: 1,
            min_rooms_per_floor: 8,
            max_rooms_per_floor: 8,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (4, 4),
            adjacent_rooms_allowed: true,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_ok());

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 5,
            max_rooms_per_floor: 5,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (3, 3),
            adjacent_rooms_allowed: false,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 5,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (3, 3),
            adjacent_rooms_allowed: false,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (4, 4),
            adjacent_rooms_allowed: false,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (3, 4),
            adjacent_rooms_allowed: false,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (4, 3),
            adjacent_rooms_allowed: false,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 5,
            max_rooms_per_floor: 5,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (4, 4),
            adjacent_rooms_allowed: true,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 5,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (4, 4),
            adjacent_rooms_allowed: true,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (6, 6),
            adjacent_rooms_allowed: true,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (5, 6),
            adjacent_rooms_allowed: true,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);

        let res = Dungeon::generate_with_params(GenerateDungeonParams {
            dimensions: (10, 10),
            num_floors: 1,
            min_rooms_per_floor: 4,
            max_rooms_per_floor: 4,
            min_room_dimensions: (1, 1),
            max_room_dimensions: (6, 5),
            adjacent_rooms_allowed: true,
            ..GenerateDungeonParams::default()
        });
        assert!(res.is_err());
        assert_eq!(res.unwrap_err(), TatamiError::TooSmall);
    }
}
