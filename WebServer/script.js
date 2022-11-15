var speedSlider = document.getElementById("speedSlider");
var speedSliderText = document.getElementById("speedSliderText");

const MoveURL = "/move";
var MotorSpeed = 10;

async function MoveRobot(dir) {
    const obj = {
        Direction: dir,
        Speed: MotorSpeed
    };

    return await fetch(MoveURL, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(obj)
    });
}

speedSlider.onmouseup = speedSlider.ontouchend = function () {
    speedSliderText.innerHTML = MotorSpeed = parseInt(this.value);
    MoveRobot('stop');
}
