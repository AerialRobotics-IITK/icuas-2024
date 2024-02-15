#Additional ROS Scripts

waitForPlants() {
  until rostopic list | grep -q "/red/plants_beds"; do
    sleep 1
  done
  echo "All plant beds spawned!"
}

waitForCount() {
  until rostopic list | grep -q "/fruit_count"; do
    sleep 1
  done
  echo "Fruit Count generated"
}

timeEvent() {
  waitForPlants
  start=`date +%s.%N`
  echo "[INFO] $start : Starting timer!"

  waitForCount
  end=`date +%s.%N`
  echo "[INFO] $end : Ending timer!"

  runtime=$( echo "($end - $start)" | bc -l) 
  runtime=$( printf %d $runtime 2> /dev/null ) 

  echo "Time elapsed in the entire mission : $(($runtime / 60))min $(($runtime % 60))sec"
}
