function Robot() {
    const maxHeight = 1;
    const minHeight = 0.3;
    const heightAdjustmentSpeed = .05;
    let height = 1;

    const moveUp = (speed = heightAdjustmentSpeed) => {
        if(speed <= 0){
            throw Error("Invalid speed for 'moveUp' selected.");
        }
        height += speed;
        if(height > maxHeight){
            height = maxHeight;
        }
        // TODO: Update height
    }
    const moveDown = (speed = heightAdjustmentSpeed) => {
        if(speed <= 0){
            throw Error("Invalid speed for 'moveDown' selected.");
        }
        height -= speed;
        if(height < minHeight){
            height = minHeight;
        }
        // TODO: Update height
    }
}

export default Robot;