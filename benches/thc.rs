use byteorder::{LittleEndian, ReadBytesExt};
use criterion::{
    criterion_group, criterion_main, AxisScale, BenchmarkId, Criterion,
    PlotConfiguration,
};
use h3o::CellIndex;
use std::{collections::HashMap, fs, io::Cursor, mem::size_of};

/// Initialize a new benchmark group with logarithmic axis scale.
macro_rules! new_benchmark_group {
    ($c:ident, $name:literal) => {{
        let plot_config =
            PlotConfiguration::default().summary_scale(AxisScale::Logarithmic);
        let mut group = $c.benchmark_group($name);
        group.plot_config(plot_config);
        group
    }};
}

/// Read a set of H3 cells serialized as a litte-endian list of u64.
fn read_binary_cell_set(path: &str) -> Vec<CellIndex> {
    let bytes = fs::read(path).expect("cell set file");
    let size = bytes.len() / size_of::<u64>();
    let mut cells = vec![0; size];

    Cursor::new(bytes)
        .read_u64_into::<LittleEndian>(&mut cells)
        .expect("H3 cells");

    cells
        .into_iter()
        .map(|value| CellIndex::try_from(value).expect("valid cell"))
        .collect()
}

fn compress(c: &mut Criterion) {
    let inputs = [10, 100, 1000, 10_000, 100_000];
    let values = inputs.iter().fold(HashMap::new(), |mut acc, key| {
        let filename = format!(
            "{}/samples/inputs/{}.u64",
            env!("CARGO_MANIFEST_DIR"),
            key
        );
        let cells = read_binary_cell_set(&filename);

        acc.insert(key, cells);
        acc
    });

    let mut group = new_benchmark_group!(c, "Compress");
    for input in inputs {
        group.bench_with_input(
            BenchmarkId::new("THC", input),
            &input,
            |b, input| {
                let values = &values[&input];
                b.iter(|| {
                    let mut buff = Cursor::new(vec![]);
                    thc::compress(&mut buff, values.iter().copied())
                });
            },
        );
    }
    group.finish();
}

fn decompress(c: &mut Criterion) {
    let inputs = [10, 100, 1000, 10_000, 100_000];
    let values = inputs.iter().fold(HashMap::new(), |mut acc, input| {
        let filename = format!(
            "{}/samples/outputs/{}.cht",
            env!("CARGO_MANIFEST_DIR"),
            input
        );
        let key = format!("{input}.cht");
        let cells = fs::read(filename).expect("THC file");
        acc.insert(key, cells);

        acc
    });

    let mut group = new_benchmark_group!(c, "Decompress");

    for input in inputs {
        group.bench_with_input(
            BenchmarkId::new("THC", input),
            &input,
            |b, input| {
                let key = format!("{input}.cht");
                let bytes = &values[&key];
                b.iter(|| {
                    thc::decompress(bytes.as_slice())
                        .collect::<Result<Vec<_>, _>>()
                        .expect("valid input")
                });
            },
        );
    }
    group.finish();
}

criterion_group!(benches, compress, decompress);
criterion_main!(benches);
