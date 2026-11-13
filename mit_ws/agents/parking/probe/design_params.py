cfg = ParkingConfig(radius_margin=1.05, clearance=0.3)
Rm = cfg.min_radius
course = json.load(open('/data/current/course.json'))
ms = {m['type']: Mission.from_json(m) for m in course['course']['missions'] if 'slot' in m}
pp = dict(radii=[Rm], final_lateral=[0.6], final_back=[1.0, 1.15],
          start_lateral=[0.0, -1.0], shift_radius=12.0, straight_before_stop=0.0,
          exit_radius=Rm * 1.1, hold_s=3.3, cost_per_radius=0.0)
rp = dict(entry_radii=[Rm, 5.5], turn_radii=[Rm], final_back=[0.0], passage_offsets=[0.0],
          final_lateral=0.0, straight_before=0.0, straight_after_turn=0.0, exit_turn_radius=Rm, exit_radius=Rm * 1.2,
          exit_passage_offset=0.0, hold_s=3.3)
free = {'parallel_parking': [(-4.0, 4.0, -2.05, 2.5)],
        'rear_parking': [(4.2, 9.2, -11.0, -2.0), (-2.8, 16.2, -2.475, 2.525)]}
