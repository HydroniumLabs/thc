//! The Compressed H3 Tree codec (CHT).
//!
//! # Encoding
//!
//! A cell index can be seen as a path into a tree, where the base cell is the
//! root node and each resolution represent a level.
//!
//! While this is one of the strong point of H3, allowing efficient
//! implementation of a lot of common operation (e.g. get the parent of a cell),
//! from a storage point of view this is a lot of redundancy that we could shave
//! of.
//!
//! That's the core of the CHT codec: decompose the indexes into paths and
//! encode them into a tree stored linearly (using van Emde Boas layout). Add
//! pinch of bitmap for compact encoding of intermediate nodes, and tada: you
//! get a Compressed H3 Tree
//!
//! # Decoding
//!
//! The compressed payload is composed of four types of objects:
//! - the base cells, i.e. roots, encoded on 8-bit (7 bit for the value + a
//!   1-bit tag).
//! - the intermediate nodes, each containing up to 7 children (6 in practice,
//!   because full nodes are compacted), encoded on 8-bit ( 7-bit bitmap + a
//!   1-bit tag).
//! - the leaf markers: a single 0 bit.
//! - an end of tree marker: a 8-bit value with all bit set.
//!
//! From each root, walk through the tree using a DFS traversal, keeping track
//! of the path until we reach a leaf marker. At that point, we can rebuild the
//! corresponding cell index, backtrack and resume the traversal. Rince and
//! repeat until we reach the `end-of-tree` marker

use crate::DecodingError;
use bitvec::prelude::*;
use h3o::{CellIndex, Resolution};
use std::io::{self, Write};

// -----------------------------------------------------------------------------

/// End of tree marker.
///
/// Used in the serialized form, to mark the end of the serialized data.
///
/// `0` is a safe value because empty node are not encoded.
const END_OF_TREE_MARKER: u8 = 0;

/// Size (in bits) of a tree node (either a base cell or directions).
/// Either:
/// - a root: one 7 bit-value
/// - a child: seven 1-bit values
const NODE_BITSIZE: usize = 7;

/// Longest path possible in the compressed H3 tree.
/// Set to `max H3 resolution + 1` to account for the base cell.
const MAX_PATH_LEN: usize = 15 + 1;

/// Average number (rounded up) of bit/H3 cell index in a compressed H3 tree.
/// Experiments have shown that we used 5.59 bit/cell index in 75% of cases.
const BITRATE: usize = 6;

// -----------------------------------------------------------------------------

/// Encodes a sorted stream of unique cell indexes into a compressed H3 tree.
///
/// During the encoding phase, each cell index is decomposed into a list of base
/// cell + directions that represents the path to the cell index in the tree.
/// The encoder keep track of the current path at all time, and uses it to
/// insert leaf marker at the appropriate locations.
/// Since the input stream of cell indexes is sorted, siblings are serialized
/// together and we can maximize common path sharing.
///
/// A little step-by-step example may help.
///
/// # Example
///
/// Let's say we want to encode the following cell indexes:
///
/// ```text
/// [0x8c2bae305336bff, 0x8c2bae305336dff, 0x862bae30fffffff]
/// ```
///
/// We start with an empty path and and empty tree.
///
/// ```text
/// path = []
/// tree = []
/// ```
///
/// We then process the first index `0x8c2bae305336bff` which can be
/// decomposed into the following list of base cell + directions:
///
/// ```text
/// 21, 6, 5, 6, 1, 4, 0, 5, 1, 4, 6, 6, 5
/// ```
///
/// Which generates the following path:
///
/// ```text
/// path = [21, 6, 5, 6, 1, 4, 0, 5, 1, 4, 6, 6, 5]
/// ```
///
/// And the following tree:
///
/// ```text
/// 1, 0,1,1,0,1,0,0, // Base cell (Base cell 21).
/// 1, 0,0,0,0,0,0,1, // Level 1   (Direction 6).
/// 1, 0,0,0,0,0,1,0, // Level 2   (Direction 5).
/// 1, 0,0,0,0,0,0,1, // Level 3   (Direction 6).
/// 1, 0,1,0,0,0,0,0, // Level 4   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 5   (Direction 4).
/// 1, 1,0,0,0,0,0,0, // Level 6   (Direction 0).
/// 1, 0,0,0,0,0,1,0, // Level 7   (Direction 5).
/// 1, 0,1,0,0,0,0,0, // Level 8   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 9   (Direction 4).
/// 1, 0,0,0,0,0,0,1, // Level 10  (Direction 6).
/// 1, 0,0,0,0,0,0,1, // Level 11  (Direction 6).
/// 1, 0,0,0,0,0,1,0, // Level 12  (Direction 5).
/// 0,                // Leaf marker.
/// ```
///
/// We can learn a few things here.
///
/// First, the leaf node are marked by a single bit set to 0.
///
/// Then, we see that tree nodes are always encoded on 8 bits.
/// They all start with a 1-bit tag indicating the presence of a node.
/// Then, the next 7 bit are either:
/// - a root node that encode the base cell value (the fact that we encode base
///   cell+1, i.e. write 22 for cell 21, is an implementation detail)
/// - an intermediate node that stores the children in a bitmap (i.e the bit `i`
///   is set if the child direction `i` is present in the tree).
///
/// Now, the encoder moves onto the next index: `0x8c2bae305336dff`
///
/// which represents:
///
/// ```text
/// 21, 6, 5, 6, 1, 4, 0, 5, 1, 4, 6, 6, 6
/// ```
///
/// By comparing with the current path, we notice that only the last direction
/// is different. Thus we can jump to the last node and set the bit for the
/// direction 6.
///
/// The path is updated to:
///
/// ```text
/// path = [21, 6, 5, 6, 1, 4, 0, 5, 1, 4, 6, 6, 6]
/// ```
///
/// And the tree is updated to:
///
/// ```text
/// 1, 0,1,1,0,1,0,0, // Base cell (Base cell 21).
/// 1, 0,0,0,0,0,0,1, // Level 1   (Direction 6).
/// 1, 0,0,0,0,0,1,0, // Level 2   (Direction 5).
/// 1, 0,0,0,0,0,0,1, // Level 3   (Direction 6).
/// 1, 0,1,0,0,0,0,0, // Level 4   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 5   (Direction 4).
/// 1, 1,0,0,0,0,0,0, // Level 6   (Direction 0).
/// 1, 0,0,0,0,0,1,0, // Level 7   (Direction 5).
/// 1, 0,1,0,0,0,0,0, // Level 8   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 9   (Direction 4).
/// 1, 0,0,0,0,0,0,1, // Level 10  (Direction 6).
/// 1, 0,0,0,0,0,0,1, // Level 11  (Direction 6).
/// 1, 0,0,0,0,0,1,1, // Level 12  (Direction 5 and 6). <-- updated
/// 0,                // Leaf marker.
/// 0,                // Leaf marker.                   <-- inserted
/// ```
///
/// Only two changes:
/// - updating the bitmap of the intermediate node (at level 12) to register the
///   new direction.
/// - adding a new leaf marker.
///
/// Finally, we move to the last index: `0x862bae30fffffff`
///
/// which contains:
///
/// ```text
/// 21, 6, 5, 6, 1, 4, 1
/// ```
///
/// This time, the path diverges at and earlier level since the cell indexes
/// have a different resolution.
/// Thus, we backtrack into the tree up to the level where the tree forked and
/// we register the new node.
///
/// The tree becomes:
///
/// ```text
/// 1, 0,1,1,0,1,0,0, // Base cell (Base cell 21).
/// 1, 0,0,0,0,0,0,1, // Level 1   (Direction 6).
/// 1, 0,0,0,0,0,1,0, // Level 2   (Direction 5).
/// 1, 0,0,0,0,0,0,1, // Level 3   (Direction 6).
/// 1, 0,1,0,0,0,0,0, // Level 4   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 5   (Direction 4).
/// 1, 1,1,0,0,0,0,0, // Level 6   (Direction 0 and 1). <-- updated
/// 1, 0,0,0,0,0,1,0, // Level 7   (Direction 5).
/// 1, 0,1,0,0,0,0,0, // Level 8   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 9   (Direction 4).
/// 1, 0,0,0,0,0,0,1, // Level 10  (Direction 6).
/// 1, 0,0,0,0,0,0,1, // Level 11  (Direction 6).
/// 1, 0,0,0,0,0,1,1, // Level 12  (Direction 5 and 6).
/// 0,                // Leaf marker.
/// 0,                // Leaf marker.
/// 0,                // Leaf marker.                   <-- inserted
/// ```
///
/// And since we've reach the end of the input, we finalize the encoding by
/// adding an `end-of-tree` marker.
///
/// The final tree is:
///
/// ```text
/// 1, 0,1,1,0,1,0,0, // Base cell (Base cell 21).
/// 1, 0,0,0,0,0,0,1, // Level 1   (Direction 6).
/// 1, 0,0,0,0,0,1,0, // Level 2   (Direction 5).
/// 1, 0,0,0,0,0,0,1, // Level 3   (Direction 6).
/// 1, 0,1,0,0,0,0,0, // Level 4   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 5   (Direction 4).
/// 1, 1,1,0,0,0,0,0, // Level 6   (Direction 0 and 1).
/// 1, 0,0,0,0,0,1,0, // Level 7   (Direction 5).
/// 1, 0,1,0,0,0,0,0, // Level 8   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 9   (Direction 4).
/// 1, 0,0,0,0,0,0,1, // Level 10  (Direction 6).
/// 1, 0,0,0,0,0,0,1, // Level 11  (Direction 6).
/// 1, 0,0,0,0,0,1,1, // Level 12  (Direction 5 and 6).
/// 0,                // Leaf marker.
/// 0,                // Leaf marker.
/// 0,                // Leaf marker.
/// 1, 0,0,0,0,0,0,0, // EOT (End of Tree) marker.        <-- inserted
/// ```
///
/// This EOT marker is necessary because we use a bit-level encoding with a
/// byte-level serialization: we can have trailing bits in the last byte that
/// we should ignore during the decoding, hence the need for a stop signal for
/// the decoder.
///
/// See encoding unit tests for more examples.
///
/// # Preconditions
///
/// The stream of cell indexes must be sorted and without duplicates.
pub fn encode<W: Write>(
    writer: &mut W,
    cells: impl IntoIterator<Item = CellIndex>,
) -> Result<(), io::Error> {
    let iter = cells.into_iter();

    // Preallocate the tree with an estimated size from iterator size hint.
    let (lower_bound, upper_bound) = iter.size_hint();
    let estimated_size = upper_bound.unwrap_or(lower_bound) * BITRATE;
    let mut tree = <BitVec<u8, Lsb0>>::with_capacity(estimated_size);

    // Current path in the tree.
    let mut path: Vec<(Step, usize)> = Vec::with_capacity(MAX_PATH_LEN);
    // XXX: push a sentinel (invalid base cell) to avoid a branch in `add_cell`.
    // Will be replaced at the first iteration, so it's safe.
    path.push((Step::BaseCell(124), 0));

    // Let's get started!
    for index in iter {
        for (level, cell) in steps_from_cell_index(index).enumerate() {
            match path.get(level) {
                // If branch diverged:
                // - update path, clear nodes below the current level.
                // - insert the new cell.
                Some(&(current_cell, _)) => {
                    if cell != current_cell {
                        path.truncate(level + 1);
                        add_cell(&mut tree, &mut path, Some((cell, level)));
                    }
                }
                // New node in the current branch, push it.
                None => add_cell(&mut tree, &mut path, Some((cell, level))),
            }
        }
        // Insert the leaf node marker.
        add_cell(&mut tree, &mut path, None);
    }

    // Finalize the serialized data with the EOT marker.
    tree.extend_from_raw_slice(&[tag(END_OF_TREE_MARKER)]);

    // Write the tree.
    writer.write_all(tree.as_raw_slice())
}

/// Adds a cell into the tree and update the path accordingly.
fn add_cell(
    tree: &mut BitVec<u8, Lsb0>,
    path: &mut Vec<(Step, usize)>,
    new_entry: Option<(Step, usize)>,
) {
    match new_entry {
        // Push a new base cell (starts a new sub-tree).
        Some((cell @ Step::BaseCell(id), _)) => {
            // Index 0 always valid thanks to the sentinel.
            path[0] = (cell, tree.len());
            // XXX: we need to shift the base cell range to [1; 122],
            // otherwise base cell 0 has the same bit pattern as EOT marker.
            tree.extend_from_raw_slice(&[tag(id + 1)]);
        }
        // Push a new cell (update the current branch or start a new one).
        Some((direction @ Step::Direction(id), level)) => {
            if let Some(&mut (ref mut current_cell, index)) =
                path.get_mut(level)
            {
                *current_cell = direction;
                // +1 to skip the node tag.
                tree.set(index + 1 + usize::from(id), true);
            } else {
                path.push((direction, tree.len()));
                tree.extend_from_raw_slice(&[tag(1 << id)]);
            }
        }
        // Push leaf marker.
        None => tree.push(false),
    }
}

/// Returns a tagged version of `value`.
const fn tag(value: u8) -> u8 {
    value << 1 | 1
}

// -----------------------------------------------------------------------------

/// Decodes a compressed H3 tree into a stream of sorted cell indexes.
///
/// During the decoding phase, we process the input as a stream of objects,
/// where an object can be either:
/// - a tree node: we push it into the path
/// - a leaf marker: we build a cell index from the path
/// - an end-of-tree marker: we're done
///
/// There is two main steps in the decoding process:
/// - extracting the next object from the bits stream
/// - building an index from a path and maintaining the path
///
/// The object extraction is fairly simple, we first read the 1-bit tag then:
/// - if it's 0 we have a leaf and build an index from the path
/// - if it's 1, we have to read the next 7 bit and interpret them either are a
///   cell to push to the path or the end of the stream.
///
/// The index building involves combining the path components into a valid cell
/// index AND to update the path to remove the current index from it (otherwise
/// we'll be stuck in a loop).
/// Updating the path is a matter a clearing the bits that encode the current
/// index.
///
/// # Example
///
/// Starting with the following tree:
///
/// ```text
/// 1, 0,1,1,0,1,0,0, // Base cell (Base cell 21).
/// 1, 0,0,0,0,0,0,1, // Level 1   (Direction 6).
/// 1, 0,0,0,0,0,1,0, // Level 2   (Direction 5).
/// 1, 0,0,0,0,0,0,1, // Level 3   (Direction 6).
/// 1, 0,1,0,0,0,0,0, // Level 4   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 5   (Direction 4).
/// 1, 1,1,0,0,0,0,0, // Level 6   (Direction 0 and 1).
/// 1, 0,0,0,0,0,1,0, // Level 7   (Direction 5).
/// 1, 0,1,0,0,0,0,0, // Level 8   (Direction 1).
/// 1, 0,0,0,0,1,0,0, // Level 9   (Direction 4).
/// 1, 0,0,0,0,0,0,1, // Level 10  (Direction 6).
/// 1, 0,0,0,0,0,0,1, // Level 11  (Direction 6).
/// 1, 0,0,0,0,0,1,1, // Level 12  (Direction 5 and 6).
/// 0,                // Leaf
/// 0,                // Leaf
/// 0,                // Leaf
/// 1, 0,0,0,0,0,0,0, // EOT (End of Tree).
/// ```
///
/// The first iteration read objects repeatedly (building a path on the way)
/// until it reach either the end-of-tree marker or a leaf marker.
/// In this case we first reach a leaf marker, and at that point we have the
/// following path:
///
/// ```text
/// Root(21),
/// Node(0000001), Node(0000010), Node(0000001), Node(0100000), Node(0000100),
/// Node(1100000), Node(0000010), Node(0100000), Node(0000100), Node(0000001),
/// Node(0000001), Node(0000011)
/// ```
/// We read the base cell and then the first bit set of every node to generate the following index:
///
/// ```text
/// 21, 6, 5, 6, 1, 4, 0, 5, 1, 4, 6, 6, 5
/// ```
///
/// Then we update the path by clearing the consumed bit:
///
/// ```text
/// Root(21),
/// Node(0000001), Node(0000010), Node(0000001), Node(0100000), Node(0000100),
/// Node(1100000), Node(0000010), Node(0100000), Node(0000100), Node(0000001),
/// Node(0000001), Node(0000001)
///                ^------- updated
/// ```
///
/// We can move forward with the rest of the tree:
///
/// ```text
/// 0,                // Leaf
/// 0,                // Leaf
/// 1, 0,0,0,0,0,0,0, // EOT (End of Tree).
/// ```
///
/// We encounter a leaf marker right away and build another cell index from the
/// path.
/// This time we get:
///
/// ```text
/// 21, 6, 5, 6, 1, 4, 0, 5, 1, 4, 6, 6, 6
/// ```
///
/// Note how the last cell changed, because we updated the path in the previous
/// iteration.
///
/// Before continuing, we update the path again, but this time something
/// interesting happens:
///
/// ```text
/// Root(21),
/// Node(0000001), Node(0000010), Node(0000001), Node(0100000), Node(0000100),
/// Node(1100000), Node(0000010), Node(0100000), Node(0000100), Node(0000001),
/// Node(0000001), Node(0000000)
///                ^------- updated
/// ```
///
/// The last node of the path is 0!
///
///
/// You should realize that using a bitmap to encode the children have two
/// benefits:
/// - gives a compact (1 bit/node) and efficient (bitwise operation) way to
///   represent and access children.
/// - provides a built-in node refcount: as long as the bitmap is not zero we
///   know there is still a cell index that references it.
///
/// So, now that the node has fallen down to 0 we know that we're done with this
/// section of the tree and we can move one level up, clearing up the bit on our
/// way.
///
/// ```text
/// Root(21),
/// Node(0000001), Node(0000010), Node(0000001), Node(0100000), Node(0000100),
/// Node(1100000), Node(0000010), Node(0100000), Node(0000100), Node(0000001),
/// Node(0000000)
/// ^------- updated
/// ```
///
/// Another 0!
/// Rince and repeat until reaching a non-zero node (new tree branch to explore)
/// or the root cell (new sub-tree to decode).
///
/// In this case we end up with the following path:
///
/// ```text
/// Root(21),
/// Node(0000001), Node(0000010), Node(0000001), Node(0100000), Node(0000100),
/// Node(0100000)
/// ^------- updated
/// ```
///
/// Let's move on with the tree:
///
/// ```text
/// 0,                // Leaf
/// 1, 0,0,0,0,0,0,0, // EOT (End of Tree).
/// ```
///
/// You know the drill now: another leaf, another index to build.
///
/// ```text
/// 21, 6, 5, 6, 1, 4, 1
/// ```
///
/// Followed by another path update, and this time we go all the way up to the
/// root and ends up with an empty path.
///
/// Let's finish this by iterating on the last bits of the tree:
///
/// ```text
/// 1, 0,0,0,0,0,0,0, // EOT (End of Tree).
/// ```
///
/// end-of-tree marker, we're done: no more cell index, decoding is over.
///
/// See decoding unit tests for more examples.
pub fn decode(bytes: &[u8]) -> Iter<'_> {
    Iter::new(bytes)
}

pub struct Iter<'a> {
    tree: &'a BitSlice<u8, Lsb0>,
    position: usize,
    path: Vec<Node>,
}

impl<'a> Iter<'a> {
    /// Initialize a new compressed H3 tree iterator.
    fn new(bytes: &'a [u8]) -> Self {
        Self {
            tree: BitSlice::from_slice(bytes),
            position: 0,
            path: Vec::with_capacity(MAX_PATH_LEN),
        }
    }

    /// Read the next object from the bit slice.
    fn next_object(&mut self) -> Result<Object, DecodingError> {
        // Read the tag bit to check if the next bits are a node or a leaf.
        let is_node = self
            .tree
            .get(self.position)
            .ok_or_else(|| DecodingError::missing_tag(self.position))?;
        self.position += 1;

        if !is_node {
            return Ok(Object::Leaf);
        }

        let bits = self
            .tree
            .get(self.position..self.position + NODE_BITSIZE)
            .ok_or_else(DecodingError::not_enough_data)?;
        let bits = bits.load_le::<u8>();
        self.position += NODE_BITSIZE;

        // Look for EOT marker.
        if bits == END_OF_TREE_MARKER {
            return Ok(Object::EndOfTree);
        }

        Ok(if self.path.is_empty() {
            // Base cell (don't forget to cancel the +1 shift).
            Node::Root(bits - 1).into()
        } else {
            Node::from(Children(bits)).into()
        })
    }

    /// Removes from the path the current cell index.
    fn remove_index_from_path(&mut self) {
        // Remove the current index, if the node is exhausted then we go up the
        // hierarchy, 'til the root if necessary.
        for i in (0..self.path.len()).rev() {
            #[allow(clippy::match_on_vec_items)] // Index is valid, cf. above.
            match self.path[i] {
                // We're back to the base cell, this whole sub-tree is done.
                Node::Root(_) => {
                    // Clear the path to start anew for the next iteration.
                    self.path.clear();
                    break;
                }
                Node::Intermediate(ref mut node) => {
                    node.remove();

                    // Siblings remain, will be processed at the next iteration.
                    if !node.is_empty() {
                        break;
                    }

                    // No more cell at this level, go up.
                    self.path.pop();
                }
            }
        }
    }
}

impl<'a> Iterator for Iter<'a> {
    type Item = Result<CellIndex, DecodingError>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut steps = Vec::with_capacity(MAX_PATH_LEN);

        loop {
            match self.next_object() {
                Ok(Object::EndOfTree) => return None,
                Ok(Object::Node(node)) => self.path.push(node),
                Ok(Object::Leaf) => {
                    // Extract the cell hierarchy from the path.
                    steps.clear();
                    for cell in self.path.iter().copied().map(From::from) {
                        steps.push(cell);
                    }
                    // Build the current cell index from the extracted steps.
                    let index = cell_index_from_steps(steps.as_slice());
                    // Prepare the next iteratiom.
                    self.remove_index_from_path();

                    return Some(index);
                }
                Err(err) => return Some(Err(err)),
            }
        }
    }
}

// -----------------------------------------------------------------------------

// If we interpret a `CellIndex` as a path, then this type represent a step.
///
/// Either we're at the start of the path (`BaseCell`) or following a
/// `Direction`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Step {
    BaseCell(u8),
    Direction(u8),
}

fn steps_from_cell_index(index: CellIndex) -> impl Iterator<Item = Step> {
    std::iter::once(Step::BaseCell(index.base_cell().into())).chain(
        Resolution::range(Resolution::One, index.resolution()).map(
            move |resolution| {
                Step::Direction(
                    index.direction_at(resolution).expect("direction").into(),
                )
            },
        ),
    )
}

fn cell_index_from_steps(steps: &[Step]) -> Result<CellIndex, DecodingError> {
    // Default cell index (resolution 0, base cell 0).
    let mut index = h3o_bit::DEFAULT_CELL_INDEX;

    let (base_cell, directions) = match steps {
        &[head, ref tail @ ..] => {
            if tail.len() > Resolution::Fifteen.into() {
                return Err(DecodingError::bad_index(
                    "too many steps in path",
                    None,
                ));
            }
            (head, tail)
        }
        _ => return Err(DecodingError::bad_index("empty path", None)),
    };

    // `directions.len() <= Resolution::Fifteen`, checked above.
    #[allow(clippy::cast_possible_truncation)]
    let resolution = directions.len() as u8;
    index = h3o_bit::set_resolution(index, resolution);

    // Set base cell.
    if let Step::BaseCell(cell) = base_cell {
        index = h3o_bit::set_base_cell(index, cell);
    } else {
        // Either we have no step (empty path error above) or we have at least
        // one and thus the first one is a base cell by definition.
        unreachable!("missing base cell");
    }

    // Set directions.
    for (i, step) in directions.iter().enumerate() {
        if let Step::Direction(direction) = *step {
            // `directions.len()`, and thus `i`, is 15 at most which fit in u8.
            #[allow(clippy::cast_possible_truncation)]
            let resolution = (i + 1) as u8; // Directions start at res 1.

            index = h3o_bit::set_direction(index, resolution, direction);
        } else {
            // Only the first step can be decoded as a base cell.
            unreachable!("more than one base cell");
        }
    }

    CellIndex::try_from(index).map_err(|err| {
        DecodingError::bad_index("invalid cell index", Some(err))
    })
}

// -----------------------------------------------------------------------------

/// A packed list of children cells, encoded as bitmap.
#[derive(Clone, Copy, Debug)]
struct Children(u8);

impl Children {
    /// Returns the first child of the node.
    const fn peek(self) -> u8 {
        self.first_child_index()
    }

    /// Removes the first child of the node.
    fn remove(&mut self) {
        // Clear the corresponding bit.
        self.0 &= !(1 << self.first_child_index());
    }

    /// Checks if there are still children in the list.
    const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Returns the index of the first child, if any.
    #[allow(clippy::cast_possible_truncation)] // 64 zeros max, can't overflow.
    const fn first_child_index(self) -> u8 {
        // Remember that a node with two children, 1 and 4 looks like this:
        //     0b00010010
        // Thus, we need to count the number of trailing zeros to get the index
        // of the first non-zero bit, i.e. the index of the first child.
        self.0.trailing_zeros() as u8
    }
}

impl From<Children> for Node {
    fn from(value: Children) -> Self {
        Self::Intermediate(value)
    }
}

/// A node from the compressed H3 tree.
#[derive(Clone, Copy, Debug)]
enum Node {
    /// A root node.
    Root(u8),
    /// An intermediate node, holding up to 7 children.
    Intermediate(Children),
}

impl From<Node> for Object {
    fn from(value: Node) -> Self {
        Self::Node(value)
    }
}

impl From<Node> for Step {
    fn from(value: Node) -> Self {
        match value {
            Node::Root(cell) => Self::BaseCell(cell),
            Node::Intermediate(children) => Self::Direction(children.peek()),
        }
    }
}

/// A serialized object.
#[derive(Clone, Copy, Debug)]
enum Object {
    /// A tree node.
    Node(Node),
    /// A leaf marker.
    Leaf,
    /// EOT marker.
    EndOfTree,
}

// -----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    macro_rules! cells {
        ($($x: expr),*) => {{
            vec![
                $(CellIndex::try_from($x).expect("valid cell"),)*
            ]
        }}
    }

    fn run_encode(input: &[u64]) -> BitVec<u8, Lsb0> {
        let mut buffer = Cursor::new(vec![]);
        let cells = input
            .iter()
            .map(|value| CellIndex::try_from(*value).expect("valid cell"));

        encode(&mut buffer, cells).expect("encode");

        BitVec::from_vec(buffer.into_inner())
    }

    #[test]
    fn encode_base_cell() {
        #[rustfmt::skip]
        let expected = bitvec![
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            0,                // Leaf.
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ];
        let result = &run_encode(&[0x802bfffffffffff])[..expected.len()];

        assert_eq!(result, expected);
    }

    #[test]
    fn encode_one_cell() {
        #[rustfmt::skip]
        let expected = bitvec![
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,0,0,0,0,0,0, // Level 6   (0).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,0, // Level 12  (5).
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ];
        let result = &run_encode(&[0x8c2bae305336bff])[..expected.len()];

        assert_eq!(result, expected);
    }

    #[test]
    fn encode_siblings() {
        #[rustfmt::skip]
        let expected = bitvec![
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,0,0,0,0,0,0, // Level 6   (0).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,1, // Level 12  (5, 6).
            0,                // Leaf
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ];
        let result = &run_encode(&[0x8c2bae305336bff, 0x8c2bae305336dff])
            [..expected.len()];

        assert_eq!(result, expected);
    }

    #[test]
    fn encode_leaf_midway() {
        #[rustfmt::skip]
        let expected = bitvec![
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,1,0,0,0,0,0, // Level 6   (0, 1).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,1, // Level 12  (5, 6).
            0,                // Leaf
            0,                // Leaf
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ];
        let result = &run_encode(&[
            0x8c2bae305336bff,
            0x8c2bae305336dff,
            0x862bae30fffffff,
        ])[..expected.len()];

        assert_eq!(result, expected);
    }

    #[test]
    fn encode_two_branches() {
        #[rustfmt::skip]
        let expected = bitvec![
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,1,1,0,0,0,0, // Level 6   (0, 1, 2).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,1, // Level 12  (5, 6).
            0,                // Leaf
            0,                // Leaf
            0,                // Leaf
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,0, // Level 12  (5).
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ];
        let result = &run_encode(&[
            0x8c2bae305336bff,
            0x8c2bae305336dff,
            0x862bae30fffffff,
            0x8c2bae315336bff,
        ])[..expected.len()];

        assert_eq!(result, expected);
    }

    #[test]
    fn encode_two_roots() {
        #[rustfmt::skip]
        let expected = bitvec![
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,1,1,0,0,0,0, // Level 6   (0, 1, 2).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,1, // Level 12  (5, 6).
            0,                // Leaf
            0,                // Leaf
            0,                // Leaf
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,0, // Level 12  (5).
            0,                // Leaf
            1, 1,1,1,0,1,0,0, // Base cell (22 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 0,0,1,0,0,0,0, // Level 6   (2).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,0, // Level 12  (5).
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ];
        let result = &run_encode(&[
            0x8c2bae305336bff,
            0x8c2bae305336dff,
            0x862bae30fffffff,
            0x8c2bae315336bff,
            0x8c2dae315336bff,
        ])[..expected.len()];

        assert_eq!(result, expected);
    }

    #[test]
    fn decode_base_cell() {
        let expected_cells = cells![0x802bfffffffffff];
        #[rustfmt::skip]
        let cells = decode(bitvec![u8, Lsb0;
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            0,                // Leaf.
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ].as_raw_slice()).collect::<Result<Vec<_>, _>>().expect("valid input");

        assert_eq!(cells, expected_cells);
    }

    #[test]
    fn decode_one_cell() {
        let expected_cells = cells![0x8c2bae305336bff];
        #[rustfmt::skip]
        let cells = decode(bitvec![u8, Lsb0;
            1, 0,1,1,0,1,0,0, // base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,0,0,0,0,0,0, // Level 6   (0).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,0, // Level 12  (5).
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ].as_raw_slice()).collect::<Result<Vec<_>, _>>().expect("valid input");

        assert_eq!(cells, expected_cells);
    }

    #[test]
    fn decode_siblings() {
        let expected_cells = cells![0x8c2bae305336bff, 0x8c2bae305336dff];
        #[rustfmt::skip]
        let cells = decode(bitvec![u8, Lsb0;
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,0,0,0,0,0,0, // Level 6   (0).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,1, // Level 12  (5, 6).
            0,                // Leaf
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ].as_raw_slice()).collect::<Result<Vec<_>, _>>().expect("valid input");

        assert_eq!(cells, expected_cells);
    }

    #[test]
    fn decode_leaf_midway() {
        let expected_cells =
            cells![0x8c2bae305336bff, 0x8c2bae305336dff, 0x862bae30fffffff];
        #[rustfmt::skip]
        let cells = decode(bitvec![u8, Lsb0;
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,1,0,0,0,0,0, // Level 6   (0, 1).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,1, // Level 12  (5, 6).
            0,                // Leaf
            0,                // Leaf
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ].as_raw_slice()).collect::<Result<Vec<_>, _>>().expect("valid input");

        assert_eq!(cells, expected_cells);
    }

    #[test]
    fn decode_two_branches() {
        let expected_cells = cells![
            0x8c2bae305336bff,
            0x8c2bae305336dff,
            0x862bae30fffffff,
            0x8c2bae315336bff
        ];
        #[rustfmt::skip]
        let cells = decode(bitvec![u8, Lsb0;
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,1,1,0,0,0,0, // Level 6   (0, 1, 2).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,1, // Level 12  (5, 6).
            0,                // Leaf
            0,                // Leaf
            0,                // Leaf
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,0, // Level 12  (5).
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ].as_raw_slice()).collect::<Result<Vec<_>, _>>().expect("valid input");

        assert_eq!(cells, expected_cells);
    }

    #[test]
    fn decode_two_roots() {
        let expected_cells = cells![
            0x8c2bae305336bff,
            0x8c2bae305336dff,
            0x862bae30fffffff,
            0x8c2bae315336bff,
            0x8c2dae315336bff
        ];
        #[rustfmt::skip]
        let cells = decode(bitvec![u8, Lsb0;
            1, 0,1,1,0,1,0,0, // Base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 1,1,1,0,0,0,0, // Level 6   (0, 1, 2).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,1, // Level 12  (5, 6).
            0,                // Leaf
            0,                // Leaf
            0,                // Leaf
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,0, // Level 12  (5).
            0,                // Leaf
            1, 1,1,1,0,1,0,0, // Base cell (22 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            1, 0,0,0,0,0,0,1, // Level 3   (6).
            1, 0,1,0,0,0,0,0, // Level 4   (1).
            1, 0,0,0,0,1,0,0, // Level 5   (4).
            1, 0,0,1,0,0,0,0, // Level 6   (2).
            1, 0,0,0,0,0,1,0, // Level 7   (5).
            1, 0,1,0,0,0,0,0, // Level 8   (1).
            1, 0,0,0,0,1,0,0, // Level 9   (4).
            1, 0,0,0,0,0,0,1, // Level 10  (6).
            1, 0,0,0,0,0,0,1, // Level 11  (6).
            1, 0,0,0,0,0,1,0, // Level 12  (5).
            0,                // Leaf
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ].as_raw_slice()).collect::<Result<Vec<_>, _>>().expect("valid input");

        assert_eq!(cells, expected_cells);
    }

    #[test]
    fn decode_invalid_base_cell() {
        #[rustfmt::skip]
        let cells = decode(bitvec![u8, Lsb0;
            1, 1,1,1,1,1,1,1, // Invalid base cell.
            0,                // Leaf.
            1, 0,0,0,0,0,0,0, // EOT (End of Tree)
        ].as_raw_slice()).collect::<Result<Vec<_>, _>>();

        assert!(cells.is_err());
    }

    #[test]
    fn decode_corrupted() {
        #[rustfmt::skip]
        let cells = decode(bitvec![u8, Lsb0;
            1, 0,1,1,0,1,0,0, // base cell (21 + 1).
            1, 0,0,0,0,0,0,1, // Level 1   (6).
            1, 0,0,0,0,0,1,0, // Level 2   (5).
            0, 0,0,0,1        // corrupted level 3.
        ].as_raw_slice()).collect::<Result<Vec<_>, _>>();

        assert!(cells.is_err());
    }

    #[test]
    fn decode_empty() {
        let cells = decode(vec![].as_slice()).collect::<Result<Vec<_>, _>>();

        assert!(cells.is_err());
    }

    #[test]
    fn rountrip_full_node() {
        // This produce a node with all bit set, which used to conflict with EOT
        // marker.
        let cells = cells![
            0x8b184584a21efff,
            0x8b184584a246fff,
            0x8b184584a2a8fff,
            0x8b184584a2cbfff,
            0x8b184584a329fff,
            0x8b184584a366fff,
            0x8b184584a389fff
        ];
        let mut buffer = Cursor::new(vec![]);
        encode(&mut buffer, cells.clone()).expect("encode");
        let bytes = buffer.into_inner();
        let result = decode(&bytes)
            .collect::<Result<Vec<_>, _>>()
            .expect("valid input");

        assert_eq!(result, cells);
    }

    #[test]
    fn rountrip_base_cell_0() {
        // Base cell 0 encoding used to conflict with EOT marker.
        let cells = cells![0x8b0153a16521fff];

        let mut buffer = Cursor::new(vec![]);
        encode(&mut buffer, cells.clone()).expect("encode");
        let bytes = buffer.into_inner();
        let result = decode(&bytes)
            .collect::<Result<Vec<_>, _>>()
            .expect("valid input");

        assert_eq!(result, cells);
    }

    #[test]
    fn decode_missing_tag() {
        let bytes = [0x81, 0x23];

        let error = decode(bytes.as_slice())
            .collect::<Result<Vec<_>, _>>()
            .expect_err("invalid input");

        assert!(matches!(error, DecodingError::MissingTag(16)));
    }

    #[test]
    fn decode_truncated_input() {
        let bytes = [0x91, 0x9d, 0xe6];

        let error = decode(bytes.as_slice())
            .collect::<Result<Vec<_>, _>>()
            .expect_err("invalid input");

        assert!(matches!(error, DecodingError::NotEnoughData));
    }

    #[test]
    fn decode_cell_too_long() {
        let bytes = [
            0x27, 0x9b, 0x94, 0x94, 0x94, 0x94, 0x94, 0x94, 0x9b, 0x9b, 0x9b,
            0x9b, 0x9b, 0x7a, 0x91, 0x23, 0xe3, 0x01, 0xce,
        ];

        let error = decode(bytes.as_slice())
            .collect::<Result<Vec<_>, _>>()
            .expect_err("invalid input");

        assert!(matches!(error, DecodingError::InvalidCellIndex { .. }));
    }
}
