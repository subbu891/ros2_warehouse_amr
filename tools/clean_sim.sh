pkill -f "gz sim" || true
pkill -f "ros2 launch" || true
pkill -f parameter_bridge || true
pkill -f _ros2_daemon || true
sleep 2

# Stale controller spawner lock left behind by an interrupted launch
rm -f /tmp/ros2-control-controller-spawner.lock

ros2 daemon start > /dev/null
remaining=$(ps aux | grep -E "gz sim|parameter_bridge|controller_manager" | grep -v grep | wc -l)
echo "Simulation cleaned up, ${remaining} leftover processes"
