from deyes_stereo.stamp_pairing import BoundedStampCache, ExactStampPairCache


def test_exact_pair_cache_accepts_out_of_order_callbacks_only_at_same_stamp():
    cache = ExactStampPairCache(capacity=4, max_age_ns=100)
    assert cache.add_right(20, "info-20", 0) is None
    assert cache.add_left(19, "depth-19", 1) is None
    assert cache.add_left(20, "depth-20", 2) == (20, "depth-20", "info-20")


def test_fast_depth_does_not_discard_plane_source_pair_before_plane_arrives():
    pairs = BoundedStampCache(capacity=8, max_age_ns=100)
    planes = BoundedStampCache(capacity=8, max_age_ns=100)
    for stamp in (100, 101, 102):
        pairs.put(stamp, f"depth-info-{stamp}", stamp)
    planes.put(100, "plane-100", 4)
    assert pairs.pop(100, 5) == "depth-info-100"
    assert planes.pop(100, 5) == "plane-100"
    assert pairs.pop(102, 5) == "depth-info-102"


def test_cache_expiry_fails_closed_and_never_returns_stale_entry():
    cache = ExactStampPairCache(capacity=2, max_age_ns=10)
    assert cache.add_left(7, "depth", 0) is None
    # Adding CameraInfo after the depth cache expired cannot fabricate a pair.
    assert cache.add_right(7, "info", 11) is None
    ready = BoundedStampCache(capacity=2, max_age_ns=10)
    ready.put(7, "pair-plane", 0)
    assert ready.pop_oldest(11) is None
