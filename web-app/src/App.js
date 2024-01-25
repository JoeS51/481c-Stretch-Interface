import './App.css';
import * as React from 'react';
import Button from '@mui/material/Button';
import { ButtonGroup, Menu } from '@mui/material';
import Box from '@mui/material/Box';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import FormControl from '@mui/material/FormControl';
import Select, { SelectChangeEvent } from '@mui/material/Select';
import Stack from '@mui/material/Stack';
import ROSLIB from "roslib";
import { RosConnection, rosImageSrcString } from 'rosreact';
import { useROS } from './ros-helpers';

function App() {
  let { ros } = useROS();
  const [part, setPart] = React.useState('');
  const [currentStatus, setCurrentStatus] = React.useState("Not Connected")
    
  const handleChange = (event) => {
    setPart(event.target.value);
  };

  // console.log(ros)

  // React.useEffect(() => {
  //   const interval = setInterval(() => {
  //     setCurrentStatus(ros.isConnected)
  //   }, 100)
  //   return () => clearInterval(interval)
  // }, [ros, setCurrentStatus])
  // console.log(RosConnection)
  // console.log(ros)

  

  const moveUp = () => {
    if (part == "Base") {
      // var ros = new ROSLIB.Ros({
      //   url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
      // })
    
      var cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });

      var twist = new ROSLIB.Message({
        linear: {x: 2.0, y: 0.0, z: 0.0},
        angular: {x: 0.0, y:0.0, z: 0.0}
      });

      cmdVelTopic.publish(twist);

      console.log("moved base")
    } else {
      console.log("not in the correct part")
    }
  }

  const moveLeft = () => {
    if (part == "Base") {
      var ros = new ROSLIB.Ros({
        url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
      })
    
      var cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });

      var twist = new ROSLIB.Message({
        linear: {x: 0.0, y: 0.0, z: 0.0},
        angular: {x: 0.0, y:0.0, z: 0.5}
      });

      cmdVelTopic.publish(twist);

      console.log("moved base")
    } else {
      console.log("not in the correct part")
    }
  }

  const moveRight = () => {
    if (part == "Base") {
      var ros = new ROSLIB.Ros({
        url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
      })
    
      var cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });

      var twist = new ROSLIB.Message({
        linear: {x: 0.0, y: 0.0, z: 0.0},
        angular: {x: 0.0, y:0.0, z: -0.5}
      });

      cmdVelTopic.publish(twist);

      console.log("moved base")
    } else {
      console.log("not in the correct part")
    }
  }

  const moveDown = () => {
    if (part == "Base") {
      var ros = new ROSLIB.Ros({
        url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
      })
    
      var cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });

      var twist = new ROSLIB.Message({
        linear: {x: -2.0, y: 0.0, z: 0.0},
        angular: {x: 0.0, y:0.0, z: 0.0}
      });

      cmdVelTopic.publish(twist);

      console.log("moved base")
    } else {
      console.log("not in the correct part")
    }
  }

  return (
    <center>
    {/* <RosConnection url="wss://slinky.hcrlab.cs.washington.edu:9090" autoConnect> */}
    <RosConnection url="ws://slinky.hcrlab.cs.washington.edu:9090" autoConnect>
      <h1>CSE 481C Stretch Web Interface</h1>
      <h4>Current Status: Connected</h4>
      <header className="App-header">
      <Stack spacing={4}>
        <FormControl variant="standard" fullWidth>
          <InputLabel id="demo-simple-select-label">Select Part</InputLabel>
          <Select
            labelId="demo-simple-select-label"
            id="demo-simple-select"
            value={part}
            label="Part"
            onChange={handleChange}
          >
            <MenuItem value="Arm">Arm</MenuItem>
            <MenuItem value="Base">Base</MenuItem>
            <MenuItem value="Wrist">Wrist</MenuItem>
            <MenuItem value="Camera">Camera</MenuItem>
          </Select>
        </FormControl>
      
        <Stack>
          <Button variant="contained" onClick={moveUp}>Up</Button>
          <ButtonGroup variant="contained" aria-label="outlined button group">
            <Button onClick={moveLeft}>Left</Button>
            <Button onClick={moveRight}>Right</Button>
          </ButtonGroup>
          <Button variant="contained" onClick={moveDown}>Down</Button>
        </Stack>
        <Button variant="outlined">Stop</Button>
      </Stack>
      
      </header>
    </RosConnection>
    </center>
  );
}

export default App;
