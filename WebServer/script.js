var speedSlider = document.getElementById('speedSlider');
var speedSliderText = document.getElementById('speedSliderText');

var MotorSpeed = 10;

async function MoveRobot(dir) {
    const obj = {
        Direction: dir,
        Speed: MotorSpeed
    };

    return await fetch('?command=MoveRobot', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(obj)
    });
}

async function OnCommand(cmd) {
    return await fetch(`?command=${cmd}`)
}

speedSlider.onmouseup = speedSlider.ontouchend = function () {
    speedSliderText.innerHTML = MotorSpeed = parseInt(this.value);
    MoveRobot('stop');
}
