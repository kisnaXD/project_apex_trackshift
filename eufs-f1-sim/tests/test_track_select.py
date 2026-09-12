"""Generic track attachment and grid geometry contracts."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / "overlay" / "eufs_racecar"))

from eufs_racecar import track_select
from eufs_racecar.grid_geometry import GridMapAdapter, build_grid_poses


def _synthetic_share(tmp_path: Path):
    (tmp_path / "worlds").mkdir()
    (tmp_path / "csv").mkdir()
    (tmp_path / "models" / "new_loop").mkdir(parents=True)
    (tmp_path / "worlds" / "new_loop.world").write_text("<sdf/>")
    (tmp_path / "models" / "new_loop" / "model.sdf").write_text("<sdf/>")
    (tmp_path / "csv" / "new_loop.csv").write_text(
        "tag,x,y,direction,x_variance,y_variance,xy_covariance\n"
        "car_start,0,0,0,0,0,0\n"
    )
    return tmp_path


def test_discovery_accepts_a_new_standard_track(tmp_path, monkeypatch):
    share = _synthetic_share(tmp_path)
    monkeypatch.setattr(track_select, "get_package_share_directory", lambda _: str(share))

    assert track_select.available_tracks() == ("new_loop",)
    resolved = track_select.resolve_track("new_loop")
    assert resolved["name"] == "new_loop"
    assert resolved["x"] == 0.0
    assert resolved["boundary_strips"] == ""


def test_grid_adapter_projects_spawn_without_a_guessed_back_offset(tmp_path):
    centerline = tmp_path / "centerline.csv"
    centerline.write_text(
        "s_m,x_m,y_m,yaw_rad\n"
        "0,0,0,0\n10,10,0,1.57079632679\n"
        "20,10,10,3.14159265359\n30,0,10,-1.57079632679\n"
    )
    assets = {
        "name": "new_loop",
        "centerline": str(centerline),
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
    }
    adapter = GridMapAdapter.from_assets(assets)
    assert adapter.nominal_spawn_s() is None
    poses = build_grid_poses(assets, 4)
    assert len(poses) == 4
    assert poses[0][0] == 0.0
    assert poses[0][1] == -2.5


def test_cota_launch_pose_comes_from_attachment_metadata(monkeypatch):
    share = Path(__file__).parents[1] / "overlay" / "eufs_tracks"
    monkeypatch.setattr(track_select, "get_package_share_directory", lambda _: str(share))
    assets = track_select.resolve_track("cota")
    assert (assets["x"], assets["y"], assets["yaw"]) == (
        -3.904850706197952,
        3.1228419364078146,
        -0.6745807971885812,
    )
    assert assets["spawn_arclength_back_m"] == 5.0
