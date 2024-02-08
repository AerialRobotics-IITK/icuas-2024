#Additional ROS Scripts

waitForPlants() {
  until rostopic list | grep -q "/red/plants_beds"; do
    sleep 1
  done
  echo "All plant beds spawned!"
}

waitForCount() {
  until rostopic list | grep -q "/red/fruit_count"; do
    sleep 1
  done
  echo "Fruit Count generated"
}

timeEvent() {
  waitForPlants
  start=`date +%s.%N`
  waitForCount
  end=`date +%s.%N`

  runtime=$( echo "($end - $start)" | bc -l) 
  runtime=$( printf %d $runtime 2> /dev/null ) 

  echo "Elapsed: $(($runtime / 60))min $(($runtime % 60))sec"

}
