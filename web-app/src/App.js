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
import { RosConnection } from 'rosreact';
import { useROS } from './ros-helpers';

function App() {
  let { ros } = useROS();
  const [part, setPart] = React.useState('');
  const [currentStatus, setCurrentStatus] = React.useState(ros.isConnected)
    
  const handleChange = (event) => {
    setPart(event.target.value);
  };

  React.useEffect(() => {
    const interval = setInterval(() => {
      setCurrentStatus(ros.isConnected)
    }, 100)
    return () => clearInterval(interval)
  }, [ros, setCurrentStatus])

  console.log(RosConnection)

  return (
    <center>
    <RosConnection url="wss://slinky.hcrlab.cs.washington.edu:9090" autoConnect>
      <h1>CSE 481C Stretch Web Interface</h1>
      <h4>Current Status: {currentStatus == "" ? "Not Connected" : currentStatus}</h4>
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
            <MenuItem value={10}>Arm</MenuItem>
            <MenuItem value={20}>Base</MenuItem>
            <MenuItem value={30}>Wrist</MenuItem>
            <MenuItem value={40}>Camera</MenuItem>
          </Select>
        </FormControl>
      
        <Stack>
          <Button variant="contained">Up</Button>
          <ButtonGroup variant="contained" aria-label="outlined button group">
            <Button>Left</Button>
            <Button>Right</Button>
          </ButtonGroup>
          <Button variant="contained">Down</Button>
        </Stack>
        <Button variant="outlined">Stop</Button>
      </Stack>
      
      </header>
    </RosConnection>
    </center>
  );
}

export default App;
