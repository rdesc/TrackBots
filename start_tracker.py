from tracker.ros_tracker import ROSTracker

if __name__ == "__main__":
    vision_address = (u'224.5.23.2', 10006)
    tracker = ROSTracker("robot_tracker", vision_address)
    tracker.start()
