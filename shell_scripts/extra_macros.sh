#Additional ROS Scripts

waitForPlants() {
  until rostopic list | grep -q "/red/plants_beds"; do
    sleep 1
  done
  echo "All plant beds spawned!"
  sleep 1
}
