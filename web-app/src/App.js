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


function App() {
  const [part, setPart] = React.useState('');

  const handleChange = (event) => {
    setPart(event.target.value);
  };
  return (
    <div className="App">
      <h1>CSE 481C Stretch Web Interface</h1>
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
      </Stack>
      </header>
    </div>
  );
}

export default App;
