import './App.css';
import * as React from 'react';
import { Button, Box, ButtonGroup, Select, InputLabel, MenuItem, Radio, RadioGroup, FormControlLabel, Grid, Paper, Typography, FormControl, FormLabel } from '@mui/material';
import Stack from '@mui/material/Stack';
import AppBar from '@mui/material/AppBar';
import Tabs from '@mui/material/Tabs';
import Tab from '@mui/material/Tab';
import ROSLIB from "roslib";
import { RosConnection, rosImageSrcString } from 'rosreact';
import { useROS } from './ros-helpers';

function App() {
  let { ros } = useROS();
  const [part, setPart] = React.useState('');
  const [currentStatus, setCurrentStatus] = React.useState("Not Connected")
  const [tabValue, setTabValue] = React.useState("Automated")
  const [controlMode, setControlMode] = React.useState('Movement');

  const handleControlChange = (event) => {
    setControlMode(event.target.value);
  };
  const handleChange = (event) => {
    setPart(event.target.value);
  };
  const handleTabChange = (event, newValue) => {
    setTabValue(newValue);
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
    if (part === "Base") {
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
    if (part === "Base") {
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
    if (part === "Base") {
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
    if (part === "Base") {
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
    <RosConnection url="ws://slinky.hcrlab.cs.washington.edu:9090" autoConnect>
      <h1>CSE 481C Stretch Web Interface</h1>
      <h4>Current Status: {currentStatus}</h4>
      <AppBar position="static" sx={{ backgroundColor: '#ffffff', boxShadow: 3 }}>
        <Tabs value={tabValue} onChange={handleTabChange} aria-label="simple tabs example">
          <Tab label="Automated" value="Automated" />
          <Tab label="Manual" value="Manual" />
        </Tabs>
      </AppBar>

      {tabValue === "Automated" && (
        <Stack spacing={4}>
          <Button variant="contained" color="primary">Clean</Button>
          <Button variant="contained" color="secondary">Return Home</Button>
        </Stack>
      )}

      {tabValue === "Manual" && (
        <Paper style={{ padding: 20 }}>
          <Typography variant="h6">Manual Control</Typography>
          <div 
  style={{
    position: 'absolute', left: '50%', top: '28%',
    transform: 'translate(-50%, -50%)',
    marginTop: 5,
    margin: 6
    
}}>
          <FormControl variant="standard" fullWidth sx={{ display: 'flex', justifyContent: 'center', marginBottom: 2 }}>
            <RadioGroup row name="controlMode" value={controlMode} onChange={handleControlChange} >
              <FormControlLabel value="Movement" control={<Radio />} label="Movement" />
              <FormControlLabel value="Arm" control={<Radio />} label="Arm" />
            </RadioGroup>
          </FormControl>
          </div>
          
          <Box sx={{ display: 'flex', justifyContent: 'space-around', padding: 8 }}>
      {/* Directional pad for Movement */}
      {controlMode === "Movement" && (
       <Box sx={{ display: 'flex', justifyContent: 'center', gap: 32 }}>
       <Stack spacing={2} alignItems="center">
       <Typography variant="subtitle1">Base</Typography>
         <Button variant="contained" onClick={moveUp}>Forward</Button>
         <ButtonGroup variant="contained" aria-label="outlined button group">
           <Button onClick={moveLeft}>Left</Button>
           <Button onClick={moveRight}>Right</Button>
         </ButtonGroup>
         <Button variant="contained" onClick={moveDown}>Backward</Button>
       </Stack>
        
       <Stack spacing={2} alignItems="center">
       <Typography variant="subtitle1"> Head Camera</Typography>
         <Button variant="contained" onClick={null}>Tilt Up</Button>
         <ButtonGroup variant="contained" aria-label="outlined button group">
           <Button onClick={null}>Left</Button>
           <Button onClick={null}>Right</Button>
         </ButtonGroup>
         <Button variant="contained" onClick={null}>Tilt Down</Button>
       </Stack>
     </Box>
      )}

      {/* Directional pad for Arm */}
      {controlMode === "Arm" && (
        <Box sx={{ display: 'flex', justifyContent: 'center', gap: 32 }}>
        <Stack spacing={2} alignItems="center">
        <Typography variant="subtitle1">Arm</Typography>
          <Button variant="contained" onClick={moveUp}>Up</Button>
          <ButtonGroup variant="contained" aria-label="outlined button group">
            <Button onClick={moveLeft}>Left</Button>
            <Button onClick={moveRight}>Right</Button>
          </ButtonGroup>
          <Button variant="contained" onClick={moveDown}>Down</Button>
        </Stack>
         
        <Stack spacing={2} alignItems="center">
        <Typography variant="subtitle1">Grabber</Typography>
          <Button variant="contained" onClick={moveUp}>Out</Button>
          <ButtonGroup variant="contained" aria-label="outlined button group">
            <Button onClick={moveLeft}>Close</Button>
            <Button onClick={moveRight}>Open</Button>
          </ButtonGroup>
          <Button variant="contained" onClick={moveDown}>In</Button>
        </Stack>
      </Box>
      )}
    </Box>
  </Paper>
)}
    </RosConnection>
  </center>
  );
}

export default App;
