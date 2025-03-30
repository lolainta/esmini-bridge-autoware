import sys

sys.path.append("/esmini/scripts/udp_driver/")

from udp_osi_common import base_port, UdpSender, OSIReceiver


def print_osi_stuff(msg):

    print(
        "OSI message timestamp: {:.2f} seconds".format(
            msg.timestamp.seconds + msg.timestamp.nanos * 1e-9
        )
    )

    # Print some static content typically only available in first message
    print("{} lanes".format(len(msg.lane)))
    for i, l in enumerate(msg.lane):
        clf = l.classification
        print("  [{}] id {} type: {}".format(i, l.id.value, clf.type))
        print("    centerline:")
        for c_line in clf.centerline:
            print("    x: {:.2f} y: {:.2f}".format(c_line.x, c_line.y))

    print("{} stationary objects".format(len(msg.stationary_object)))
    for i, s in enumerate(msg.stationary_object):
        print("  [{}] id {} type {}".format(i, s.id.value, s.classification.type))
        print(
            "    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}".format(
                s.base.position.x, s.base.position.y, s.base.orientation.yaw
            )
        )

    # Print some dynamic content from the message
    print("{} moving objects".format(len(msg.moving_object)))
    for i, o in enumerate(msg.moving_object):
        print("  [{}] id {}".format(i, o.id.value))
        print(
            "    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}".format(
                o.base.position.x, o.base.position.y, o.base.orientation.yaw
            )
        )
        print(
            "    vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}".format(
                o.base.velocity.x, o.base.velocity.y, o.base.orientation_rate.yaw
            )
        )
        print(
            "    acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}".format(
                o.base.acceleration.x,
                o.base.acceleration.y,
                o.base.orientation_acceleration.yaw,
            )
        )

        lane_id = (
            o.assigned_lane_id[0].value
            if len(msg.lane) > 0 and len(o.assigned_lane_id) > 0
            else -1
        )
        left_lane_id = -1
        right_lane_id = -1
        for l in msg.lane:
            if l.id.value == o.assigned_lane_id[0].value:
                left_lane_id = (
                    l.classification.left_adjacent_lane_id[0].value
                    if len(l.classification.left_adjacent_lane_id) > 0
                    else -1
                )
                right_lane_id = (
                    l.classification.right_adjacent_lane_id[0].value
                    if len(l.classification.right_adjacent_lane_id) > 0
                    else -1
                )
                break
        print(
            "    lane id {} left adj lane id {} right adj lane id {}".format(
                lane_id, left_lane_id, right_lane_id
            )
        )


def get_ego_info(msg):
    # Print some dynamic content from the message
    print("{} moving objects".format(len(msg.moving_object)))
    for i, o in enumerate(msg.moving_object):
        print("  [{}] id {}".format(i, o.id.value))
        print(
            "    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}".format(
                o.base.position.x, o.base.position.y, o.base.orientation.yaw
            )
        )
        print(
            "    vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}".format(
                o.base.velocity.x, o.base.velocity.y, o.base.orientation_rate.yaw
            )
        )
        print(
            "    acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}".format(
                o.base.acceleration.x,
                o.base.acceleration.y,
                o.base.orientation_acceleration.yaw,
            )
        )

        lane_id = (
            o.assigned_lane_id[0].value
            if len(msg.lane) > 0 and len(o.assigned_lane_id) > 0
            else -1
        )
        left_lane_id = -1
        right_lane_id = -1
        for l in msg.lane:
            if l.id.value == o.assigned_lane_id[0].value:
                left_lane_id = (
                    l.classification.left_adjacent_lane_id[0].value
                    if len(l.classification.left_adjacent_lane_id) > 0
                    else -1
                )
                right_lane_id = (
                    l.classification.right_adjacent_lane_id[0].value
                    if len(l.classification.right_adjacent_lane_id) > 0
                    else -1
                )
                break
        print(
            "    lane id {} left adj lane id {} right adj lane id {}".format(
                lane_id, left_lane_id, right_lane_id
            )
        )
        if o.id.value == 0:
            x = o.base.position.x
            y = o.base.position.y
            vx = o.base.velocity.x
            vy = o.base.velocity.y
            ax = o.base.acceleration.x
            ay = o.base.acceleration.y
            return x, y, vx, vy, ax, ay

    print("Ego vehicle not found")
    return None, None, None, None, None, None


def main(osiReceiver):
    try:
        msg = osiReceiver.receive()
        x, y, vx, vy, ax, ay = get_ego_info(msg)
    except timeout:
        print("osiReceive Timeout")
        exit()
    except KeyboardInterrupt:
        print("Ctrl+C pressed, quit")
        exit()

    print(f"x: {x}, y: {y}, vx: {vx}, vy: {vy}, ax: {ax}, ay: {ay}")


if __name__ == "__main__":
    print("UDP Controller")
    udpSender = UdpSender(port=base_port + 0)
    osiReceiver = OSIReceiver()

    main(osiReceiver)

    exit()

    done = False
    counter = 0

    while not done and counter < 3:
        # Read OSI
        try:
            msg = osiReceiver.receive()
            print_osi_stuff(msg)
        except timeout:
            print("osiReceive Timeout")
            done = True
        except KeyboardInterrupt:
            print("Ctrl+C pressed, quit")
            done = True

        counter += 1

    # Close and quit
    udpSender.close()
    osiReceiver.close()
