import './App.css';
import * as React from 'react';
import { createTheme, ThemeProvider, Button, Box, ButtonGroup, Radio, RadioGroup, FormControlLabel, Paper, Typography, FormControl } from '@mui/material';
import Stack from '@mui/material/Stack';
import AppBar from '@mui/material/AppBar';
import Tabs from '@mui/material/Tabs';
import Tab from '@mui/material/Tab';
import ROSLIB from "roslib";
import { useROS } from './ros-helpers';
import Konva from 'konva';
import { Stage, Layer, Image } from 'react-konva';

const JOINTS = [
  "wrist_extension",
  "joint_lift",
  "joint_arm_l3",
  "joint_arm_l2",
  "joint_arm_l1",
  "joint_arm_l0",
  "joint_head_pan",
  "joint_head_tilt",
  "joint_wrist_yaw",
  "joint_wrist_pitch",
  "joint_wrist_roll",
  "joint_gripper_finger_left",
  "joint_gripper_finger_right",
]

const JOINT_LIMITS = {
  "wrist_extension": [0.05, .518],
  "joint_wrist_roll": [-2.95, 2.95],
  "joint_wrist_pitch": [-1.57, 0.57],
  "joint_wrist_yaw": [-1.37, 4.41],
  "joint_lift": [0.175, 1.05],
  "translate_mobile_base": [-30.0, 30.0],
  "rotate_mobile_base": [-3.14, 3.14],
  "joint_gripper_finger_left": [-0.37, 0.17],
  "joint_head_tilt": [-1.6, 0.3],
  "joint_head_pan": [-3.95, 1.7]
}

function App() {
  let { ros } = useROS();
  const [part, setPart] = React.useState('Movement');
  const [currentStatus, setCurrentStatus] = React.useState("Not Connected")
  const [tabValue, setTabValue] = React.useState("Automated")
  const [controlMode, setControlMode] = React.useState('Movement');
  const [cameraSubscribed, setCameraSubscribed] = React.useState("");
  const [markers, setMarkers] = React.useState("");
  const [rosConnected, setRosConnected] = React.useState();
  const [isVisible, setIsVisible] = React.useState(false);
  const [jointStates, setJointStates] = React.useState();
  const [currLocName, setCurrLocName] = React.useState("");

  const handleControlChange = (event) => {
    setControlMode(event.target.value);
    setPart(event.target.value)
  };
  const handleChange = (event) => {
    setPart(event.target.value);
  };
  const handleTabChange = (event, newValue) => {
    setTabValue(newValue);
  };
  // console.log(ros)

  React.useEffect(() => {
    console.log("in use effect")
    setRosConnected(new ROSLIB.Ros({
      url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
    }));

  }, []);

  React.useEffect(() => {
    if (rosConnected) {
      var cameraTopic = new ROSLIB.Topic({
        ros: rosConnected,
        name: '/camera/color/image_raw/compressed',
        messageType: 'sensor_msgs/msg/CompressedImage',
      });
      console.log("In camera")
      cameraTopic.subscribe((message) => {
        setCameraSubscribed("data:image/jpg;base64," + message.data)
      });

      var markerTopic = new ROSLIB.Topic({
        ros: rosConnected,
        name: '/aruco/marker_array',
        messageType: 'visualization_msgs/msg/MarkerArray',
      });

      markerTopic.subscribe((message) => {
        setMarkers(message.markers[0] ? message.markers[0].id : "")
        //console.log(message.markers[0].id)
        console.log("MARKER")
      });

      var jointStatesTopic = new ROSLIB.Topic({
        ros: rosConnected,
        name: '/stretch/joint_states',
        messageType: 'sensor_msgs/msg/JointState'
      });

      jointStatesTopic.subscribe((message) => {
        setJointStates(message.position);
        //console.log(message.position[JOINTS.indexOf("wrist_extension")])
      })
    }
  }, [rosConnected])

  // React.useEffect(() => {
  //   const interval = setInterval(() => {
  //     setCurrentStatus(ros.isConnected)
  //   }, 100)
  //   return () => clearInterval(interval)
  // }, [ros, setCurrentStatus])
  // console.log(RosConnection)
  // console.log(ros)


  const moveUp = (e) => {
    if (part === "Movement" && rosConnected) {
      // var ros = new ROSLIB.Ros({
      //   url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
      // })

      var cmdVelTopic = new ROSLIB.Topic({
        ros: rosConnected,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });

      var twist = new ROSLIB.Message({
        linear: { x: 2.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
      });
      cmdVelTopic.publish(twist);

      console.log("moved base")
    } else {
      console.log("not in the correct part")
    }
  }

  const moveLeft = () => {
    if (part === "Movement" && rosConnected) {
      // var ros = new ROSLIB.Ros({
      //   url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
      // })

      var cmdVelTopic = new ROSLIB.Topic({
        ros: rosConnected,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });

      var twist = new ROSLIB.Message({
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.5 }
      });

      cmdVelTopic.publish(twist);

      console.log("moved base")
    } else {
      console.log("not in the correct part")
    }
  }

  const moveRight = () => {
    if (part === "Movement" && rosConnected) {
      // var ros = new ROSLIB.Ros({
      //   url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
      // })

      var cmdVelTopic = new ROSLIB.Topic({
        ros: rosConnected,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });

      var twist = new ROSLIB.Message({
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: -0.5 }
      });

      cmdVelTopic.publish(twist);

      console.log("moved base")
    } else {
      console.log("not in the correct part")
    }
  }

  const moveDown = () => {
    if (part === "Movement" && rosConnected) {
      // var ros = new ROSLIB.Ros({
      //   url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
      // })

      var cmdVelTopic = new ROSLIB.Topic({
        ros: rosConnected,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });

      var twist = new ROSLIB.Message({
        linear: { x: -2.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
      });

      cmdVelTopic.publish(twist);

      console.log("moved base")
    } else {
      console.log("not in the correct part")
    }
  }
  const buttonTheme = createTheme({
    palette: {
      // Define a custom palette color
      customColor: {
        main: '#00e6e6', // Custom color
        contrastText: '#00000', // Text color against the background
      },
      StopColor: {
        main: '#e62e00', // Custom color
        contrastText: '#00000', // Text color against the background
      }
    },
  });

  const testActionServer = (joint_name, pos) => {
    // var ros = new ROSLIB.Ros({
    //   url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
    // })
    if (rosConnected) {
      console.log("lift joint")
      var jointLiftClient = new ROSLIB.ActionHandle({
        ros: rosConnected,
        name: '/stretch_controller/follow_joint_trajectory',
        actionType: 'control_msgs/action/FollowJointTrajectory',
      });

      let newPos = jointStates[JOINTS.indexOf(joint_name)] + pos;
      console.log("curr pose: " + jointStates[JOINTS.indexOf(joint_name)])
      console.log("NEW POSE: " + newPos)

      var goal = new ROSLIB.ActionGoal({
        trajectory: {
          header: {
            stamp: {
              secs: 0,
              nsecs: 0
            }
          },
          joint_names: [joint_name],
          points: [
            {
              positions: [newPos],
              time_from_start: {
                secs: 0,
                nsecs: 1
              }
            }
          ]
        }

      });

      console.log(goal);

      // goal.on('feedback', function(feedback) {
      //   console.log('Feedback: ' + feedback.sequence);
      // });

      // goal.on('result', function(result) {
      //   console.log('Final Result: ' + result.sequence);
      // });

      jointLiftClient.createClient(goal);
    }

  }

  const savePosition = () => {
    // call the callback from ros
  }

  let image = new window.Image();
  image.src = cameraSubscribed;


  return (
    <center>
      {/* <RosConnection url="ws://slinky.hcrlab.cs.washington.edu:9090" autoConnect> */}
      <h1>CSE 481C Stretch Web Interface</h1>
      <h3>Current Status: {currentStatus}</h3>
      <h3> Marker ID: {markers ? markers : "Not Found"} </h3>
      <Button onClick={testActionServer}>TestActionServer</Button>

      <AppBar position="static" sx={{ backgroundColor: '#00ff00', boxShadow: 8, alignItems: 'center', paddingBottom: '10px', paddingTop: '10px' }}>
        <Tabs value={tabValue} onChange={handleTabChange} aria-label="simple tabs example">
          <Tab label="Automated" value="Automated" />
          <Tab label="Manual" value="Manual" />
        </Tabs>
      </AppBar>

      {tabValue === "Automated" && (
        <ThemeProvider theme={buttonTheme}>
          <div>
            <label for="cars">Choose a car:</label>
            {/* Change the values of the dropdown */}
            <select name="cars" id="cars">
              <option value="volvo">Volvo</option>
              <option value="saab">Saab</option>
              <option value="mercedes">Mercedes</option>
              <option value="audi">Audi</option>
            </select>
          </div>
          <ButtonGroup >
            <ButtonGroup orientation='vertical' >
              <Button variant="contained" color="customColor" style={{ marginTop: '28px', marginBottom: '78px', width: '250px', height: '75px' }}>Clean the Room</Button>

              <Button variant="contained" color="secondary" style={{ width: '150px', height: '50px' }} >Return Home</Button>
            </ButtonGroup>
            <Button variant='contained' color="StopColor" style={{ marginLeft: '84px', marginTop: '28px', width: '350px', height: '200px' }} >Emergency Stop</Button>
          </ButtonGroup>
          
        </ThemeProvider>


      )}

      {tabValue === "Manual" && (
        <Paper style={{ padding: 20 }}>
          <Typography variant="h6">Manual Control</Typography>
          <div>
            <input type="text" value={currLocName} onChange={(e) => setCurrLocName(e.target.value)} placeholder='Enter a position name' />
            <Button variant="contained" onClick={savePosition} style={{ marginLeft: "10px" }}>Save Position</Button>
          </div>
          <div
            // style={{
            //   position: 'absolute', left: '50%', top: '28%', transform: 'translate(-50%, -50%)', marginTop: 5, margin: 6
            // }}
            style={{
              marginLeft: "45%"
            }}
          >
            <FormControl variant="standard" fullWidth sx={{ display: 'flex', justifyContent: 'center', marginBottom: 2 }}>
              <RadioGroup row name="controlMode" value={controlMode} onChange={handleControlChange} sx={{ marginTop: '35px' }}>
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
                  <Button variant="contained" onMouseDown={moveUp} style={{ width: '100px', height: '37px' }}>Forward</Button>
                  <ButtonGroup variant="contained" >
                    <Button onClick={moveLeft} style={{ marginRight: ' 32px', width: '100px', height: '37px' }}>Left</Button>
                    <Button onClick={moveRight} style={{ width: '100px', height: '37px' }}>Right</Button>
                  </ButtonGroup>
                  <Button variant="contained" onClick={moveDown} style={{ width: '100px', height: '37px' }}>Backward</Button>
                </Stack>

                <Stack spacing={2} alignItems="center">
                  <Typography variant="subtitle1"> Head Camera</Typography>
                  <Button variant="contained" onClick={() => testActionServer("joint_head_tilt", 0.1)} style={{ width: '100px', height: '37px' }}>Tilt Up</Button>
                  <ButtonGroup variant="contained" aria-label="outlined button group" >
                    <Button onClick={() => testActionServer("joint_head_pan", 0.1)} style={{ marginRight: ' 32px', width: '100px', height: '37px' }}>Left</Button>
                    <Button onClick={() => testActionServer("joint_head_pan", -0.1)} style={{ width: '100px', height: '37px' }}>Right</Button>
                  </ButtonGroup>
                  <Button variant="contained" onClick={() => testActionServer("joint_head_tilt", -0.1)} style={{ width: '115px', height: '37px' }}>Tilt Down</Button>
                </Stack>
              </Box>
            )}

            {/* Directional pad for Arm */}
            {controlMode === "Arm" && (
              <Box sx={{ display: 'flex', justifyContent: 'center', gap: 32 }}>
                <Stack spacing={2} alignItems="center">
                  <Typography variant="subtitle1">Arm</Typography>
                  <Button variant="contained" onClick={() => testActionServer("joint_lift", 0.075)} style={{ width: '100px', height: '37px' }}>Up</Button>
                  <ButtonGroup variant="contained" aria-label="outlined button group" >
                    <Button onClick={moveLeft} style={{ marginRight: ' 32px', width: '100px', height: '37px' }}>Left</Button>
                    <Button onClick={moveRight} style={{ width: '100px', height: '37px' }}>Right</Button>
                  </ButtonGroup>
                  <Button variant="contained" onClick={() => testActionServer("joint_lift", -0.075)} style={{ width: '100px', height: '37px' }}>Down</Button>
                </Stack>

                <Stack spacing={2} alignItems="center">
                  <Typography variant="subtitle1">Grabber</Typography>
                  <Button variant="contained" onClick={() => testActionServer("wrist_extension", 0.075)} style={{ width: '100px', height: '37px' }}>Out</Button>
                  <ButtonGroup variant="contained" aria-label="outlined button group">
                    <Button onClick={() => testActionServer("joint_gripper_finger_left", -0.075)} style={{ marginRight: ' 32px', width: '100px', height: '37px' }}>Close</Button>
                    <Button onClick={() => testActionServer("joint_gripper_finger_left", 0.075)} style={{ width: '100px', height: '37px' }}>Open</Button>
                  </ButtonGroup>
                  <Button variant="contained" onClick={() => testActionServer("wrist_extension", -0.075)} style={{ width: '100px', height: '37px' }}>In</Button>
                </Stack>
              </Box>
            )}
          </Box>
          <img src={cameraSubscribed} x={0} y={0} width={400} height={400} />
        </Paper>
      )}

      {/* </RosConnection> */}
    </center>
  );
}

export default App;
