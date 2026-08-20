from deyes_stereo.socl_isaac_perception_contract import (
    ISAAC_SIM_SOURCE,
    CameraInfoDescriptor,
    Header,
    ImageDescriptor,
    bridge_isaac_depth_camera_info,
    validate_physical_or_command_consumption,
)


def _depth(*, stamp=1_000_000_000, width=1280, height=720, encoding="32FC1", frame="Left_camera", source=ISAAC_SIM_SOURCE):
    return ImageDescriptor(Header(stamp, frame), width, height, encoding, source)


def _info(*, stamp=1_010_000_000, width=1280, height=720, frame="sim_camera", projection=None, source=ISAAC_SIM_SOURCE):
    return CameraInfoDescriptor(
        Header(stamp, frame), width, height,
        tuple(projection if projection is not None else [857.731902, 0.0, 640.0, 0.0, 0.0, 857.731902, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]),
        source,
    )


def test_bridge_reheaders_camera_info_to_exact_depth_header_and_preserves_source_metadata():
    result = bridge_isaac_depth_camera_info(_depth(), _info(), received_stamp_ns=1_100_000_000)
    assert result.valid
    assert result.camera_info_rect is not None
    assert result.camera_info_rect.header == Header(1_000_000_000, "Left_camera")
    assert result.camera_info_rect.width == 1280 and result.camera_info_rect.height == 720
    assert result.metadata is not None
    assert result.metadata.camera_info_original_frame_id == "sim_camera"
    assert result.metadata.original_stamp_skew_ns == 10_000_000
    assert result.metadata.simulation_validated
    assert not result.metadata.physical_validated
    assert not result.metadata.command_consumption_allowed
    assert not result.metadata.physical_consumption_allowed


def test_bridge_rejects_size_encoding_stamp_skew_projection_and_stale_inputs():
    bad = bridge_isaac_depth_camera_info(
        _depth(width=640, encoding="16UC1"),
        _info(width=1280, stamp=1_100_000_000, projection=[0.0] * 12),
        received_stamp_ns=2_000_000_000,
        max_stamp_skew_ns=20_000_000,
        max_camera_info_age_ns=100_000_000,
    )
    assert not bad.valid and bad.camera_info_rect is None
    assert set(bad.reasons) >= {
        "depth_encoding_must_be_32FC1",
        "depth_camera_info_size_mismatch",
        "camera_info_projection_invalid",
        "camera_info_stamp_skew_exceeds_limit",
        "camera_info_stale",
    }
    assert bad.metadata is not None and not bad.metadata.simulation_validated


def test_source_is_simulation_only_and_physical_consumption_is_always_rejected():
    result = bridge_isaac_depth_camera_info(_depth(source="physical_checkerboard"), _info(), received_stamp_ns=1_100_000_000)
    assert not result.valid
    assert "source_must_be_isaac_sim" in result.reasons
    allowed, reasons = validate_physical_or_command_consumption(
        bridge_isaac_depth_camera_info(_depth(), _info(), received_stamp_ns=1_100_000_000).metadata
    )
    assert not allowed
    assert reasons == ("isaac_sim_data_cannot_be_used_for_physical_or_command_consumption",)


def test_original_frame_mismatch_is_metadata_not_a_reason_to_hide_the_simulator_contract():
    result = bridge_isaac_depth_camera_info(_depth(), _info(frame="different_sim_frame"), received_stamp_ns=1_100_000_000)
    assert result.valid
    assert result.metadata is not None
    assert result.metadata.camera_info_original_frame_id == "different_sim_frame"
    assert result.camera_info_rect is not None
    assert result.camera_info_rect.header.frame_id == "Left_camera"
