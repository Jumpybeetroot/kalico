import importlib.util
import pathlib

import pytest


MODULE_PATH = (
    pathlib.Path(__file__).parents[1]
    / "klippy"
    / "extras"
    / "tmc4671_sync.py"
)
spec = importlib.util.spec_from_file_location("tmc4671_sync_under_test", MODULE_PATH)
tmc4671_sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tmc4671_sync)


@pytest.mark.parametrize(
    "sync_rate,trajectory_rate,expected_decimation",
    [
        (5000, 2500, 2),
        (10000, 5000, 2),
        (5000, 1000, 5),
        (2000, 2000, 1),
    ],
)
def test_calc_trajectory_decimation_accepts_exact_divisors(
    sync_rate, trajectory_rate, expected_decimation
):
    assert (
        tmc4671_sync.calc_trajectory_decimation(sync_rate, trajectory_rate)
        == expected_decimation
    )


@pytest.mark.parametrize(
    "sync_rate,trajectory_rate",
    [
        (5000, 3000),
        (10000, 3333),
        (5000, 0),
        (5000, 6000),
        (10000, 1),
    ],
)
def test_calc_trajectory_decimation_rejects_invalid_rates(
    sync_rate, trajectory_rate
):
    with pytest.raises(ValueError):
        tmc4671_sync.calc_trajectory_decimation(sync_rate, trajectory_rate)
