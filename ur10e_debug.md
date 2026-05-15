Some common things to debug connections when working with the UR10e.

1) If you see that no valid packet has arrived, just re-run your script.
2) "urx.urrobot.RobotException: Goal not reached but no program has been running for {} seconds." - error when you are in Automatic instead of Remote
3) "ValueError: invalid literal for int() with base 10: '?'" - error when running without resetting estop
4) "TimeoutError: timed out" - error when running software but robot not turned on and therefore no connection established - this can be confirmed by pinging the robot at 192.168.0.2

