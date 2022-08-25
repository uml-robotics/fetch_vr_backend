function estopLoop {
  echo "Please enter '1' to engage estop or '2' to disengage estop."
  select val in "Estop" "Disengage_Estop" "Exit"; do
  case $val in
    Estop )
        rostopic pub --once /enable_software_runstop std_msgs/Bool 'true'
        break
        ;;
    Disengage_Estop )
        rostopic pub --once /enable_software_runstop std_msgs/Bool 'false'
        break
        ;;
    Exit ) exit;;
  esac
  done

  estopLoop
}

estopLoop