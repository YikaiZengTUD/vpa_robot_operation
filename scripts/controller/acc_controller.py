def acc_controller(dis_ref, dis_meas,
                   dead_band=0.1,   # m: ok if we're ≥10cm farther than ref
                   slow_zone=0.10,   # m: how much "too close" maps to full stop
                   z_stop=0.35,      # m: hard stop threshold
                   min_factor=0.0):  # floor (e.g., 0.05 for crawl)
    """
    Pure function. Returns throttle scale in [0,1].
    dis_ref: desired gap (m)
    dis_meas: measured gap (m)
    """
    if dis_ref is None or dis_meas is None:
        return 1.0

    # immediate hard stop (stateless)
    if dis_meas <= z_stop:
        return min_factor

    # error: positive means too close
    e = float(dis_ref - dis_meas)

    # far enough beyond ref → full speed
    if e <= -dead_band:
        return 1.0

    # much too close → full stop
    if e >= slow_zone:
        return min_factor

    # smooth ease-out from 1 → 0 across [-dead_band, +slow_zone]
    t = (e + dead_band) / (slow_zone + dead_band)   # t∈[0,1]
    f = (1.0 - t)**2                                # quadratic
    return max(min_factor, min(1.0, f))
