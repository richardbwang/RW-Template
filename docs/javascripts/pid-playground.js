// PID playground: a simulated robot runs RW's turnToAngle with the constants you pick.
// The controller below follows src/pid.cpp and turnToAngle in src/motor-control.cpp step for
// step (10 ms updates, kD on the change per update, integral only within 3°, the same exit
// checks), so the numbers carry over to custom/src/robot-config.cpp. The robot is a simple
// model, so real robots will differ a bit.
(function () {
  "use strict";

  var ROBOTS = {
    typical: { name: "Typical robot", maxSpeed: 480, lag: 0.1, staticVolts: 0.4 },
    zippy: { name: "Light and zippy", maxSpeed: 650, lag: 0.07, staticVolts: 0.3 },
    heavy: { name: "Heavy pusher", maxSpeed: 380, lag: 0.18, staticVolts: 0.8 },
  };
  var TIME_LIMIT = 2.5; // seconds
  var STEP = 0.01; // turnToAngle's wait(10, msec)

  function simulate(opts) {
    var robot = ROBOTS[opts.robot] || ROBOTS.typical;
    var kp = opts.kp, ki = opts.ki, kd = opts.kd, target = opts.target;
    // turnToAngle's PID settings
    var threshold = 1, integralRange = 3, derivativeTolerance = threshold * 4.5;
    var smallTol = threshold, bigTol = threshold * 3, smallDur = 0.05, bigDur = 0.25;

    var heading = 0, speed = 0; // degrees, degrees per second
    var prevError = target, sumError = 0, smallStart = 0, bigStart = 0;
    var t = 0, volts = 0, arrivedAt = null, holding = false;
    var times = [0], headings = [0], voltages = [0];

    while (t < TIME_LIMIT + 0.6) {
      if (!holding) {
        if (t >= TIME_LIMIT) {
          holding = true; // timed out: turnToAngle gives up and holds
        } else {
          var error = target - heading;
          var derivative = kd * (error - prevError);
          prevError = error;
          if (Math.abs(error) >= integralRange) sumError = 0;
          else sumError += error;
          if (Math.sign(sumError) !== Math.sign(error) || Math.abs(error) <= smallTol) sumError = 0;
          volts = kp * error + ki * sumError + derivative;
          volts = Math.max(-12, Math.min(12, volts));

          var slow = Math.abs(derivative) <= derivativeTolerance;
          if (!(Math.abs(error) <= smallTol && slow)) smallStart = t;
          if (!(Math.abs(error) <= bigTol && slow)) bigStart = t;
          if (t - smallStart >= smallDur || t - bigStart >= bigDur) {
            arrivedAt = t;
            holding = true;
          }
        }
        if (holding) volts = 0;
      }
      // Ten 1 ms physics steps per control step
      for (var i = 0; i < 10; i++) {
        var goal;
        if (holding) {
          goal = 0; // brake type hold: the motors stop the robot quickly
          speed += (goal - speed) * 0.001 / 0.03;
        } else {
          var push = Math.max(0, Math.abs(volts) - robot.staticVolts) / (12 - robot.staticVolts);
          goal = Math.sign(volts) * push * robot.maxSpeed;
          // Friction holds a slow robot still when the motors push too gently
          if (push === 0 && Math.abs(speed) < 20) goal = 0;
          speed += (goal - speed) * 0.001 / robot.lag;
        }
        heading += speed * 0.001;
      }
      t = Math.round((t + STEP) * 1000) / 1000;
      times.push(t);
      headings.push(heading);
      voltages.push(volts);
    }

    var peak = 0;
    for (var j = 0; j < headings.length; j++) {
      peak = target >= 0 ? Math.max(peak, headings[j] - target) : Math.max(peak, target - headings[j]);
    }
    return {
      times: times,
      headings: headings,
      volts: voltages,
      arrivedAt: arrivedAt,
      overshoot: peak,
      finalError: target - headings[headings.length - 1],
      timeLimit: TIME_LIMIT,
    };
  }

  function verdict(r) {
    var finalErr = Math.abs(r.finalError);
    if (r.arrivedAt === null) {
      if (finalErr > 3) return ["Never got there", "It ran out of time. Raise kP, or kI if it stalls just short."];
      return ["Never settled", "It's close but keeps moving. Raise kD to calm it down."];
    }
    if (r.overshoot > 5) return ["Big overshoot", "It swings past the target. Raise kD or lower kP."];
    if (finalErr > 1.5) return ["Stopped short", "It quit before reaching the target. Raise kP or add a little kI."];
    if (r.overshoot > 1.5) return ["A little overshoot", "Close. A bit more kD should tighten it up."];
    if (r.arrivedAt > 1.2) return ["Slow but steady", "Accurate, but it takes its time. Try more kP."];
    return ["Nice and crisp", "Fast, accurate, no wobble. That's a good tune."];
  }

  // ---- Page widget -------------------------------------------------------------------

  var PRESETS = {
    rw: { label: "RW default", kp: 0.3, ki: 0, kd: 2.5 },
    hot: { label: "Too much kP", kp: 1.0, ki: 0, kd: 2.5 },
    nod: { label: "No kD", kp: 0.3, ki: 0, kd: 0 },
    shy: { label: "Too gentle", kp: 0.06, ki: 0, kd: 0.5 },
  };

  function el(tag, attrs, children) {
    var node = document.createElement(tag);
    Object.keys(attrs || {}).forEach(function (k) {
      if (k === "text") node.textContent = attrs[k];
      else node.setAttribute(k, attrs[k]);
    });
    (children || []).forEach(function (c) { node.appendChild(c); });
    return node;
  }

  var SVG = "http://www.w3.org/2000/svg";
  function svg(tag, attrs) {
    var node = document.createElementNS(SVG, tag);
    Object.keys(attrs || {}).forEach(function (k) { node.setAttribute(k, attrs[k]); });
    return node;
  }

  function mount(root) {
    var state = { kp: 0.3, ki: 0, kd: 2.5, target: 90, robot: "typical" };
    var reduceMotion = window.matchMedia && matchMedia("(prefers-reduced-motion: reduce)").matches;

    // Field with the robot, seen from above
    var field = svg("svg", { viewBox: "-60 -60 120 120", class: "pidp-field", role: "img", "aria-label": "Robot turning, seen from above" });
    for (var g = -60; g <= 60; g += 40) {
      field.appendChild(svg("line", { x1: g, y1: -60, x2: g, y2: 60, class: "pidp-tile" }));
      field.appendChild(svg("line", { x1: -60, y1: g, x2: 60, y2: g, class: "pidp-tile" }));
    }
    var targetRay = svg("line", { x1: 0, y1: 0, x2: 0, y2: -52, class: "pidp-target-ray" });
    var trail = svg("path", { class: "pidp-trail", d: "" });
    var robotG = svg("g", { class: "pidp-robot" });
    robotG.appendChild(svg("rect", { x: -15, y: -15, width: 30, height: 30, rx: 5, class: "pidp-body" }));
    robotG.appendChild(svg("rect", { x: -18, y: -12, width: 4, height: 24, rx: 1.5, class: "pidp-wheel" }));
    robotG.appendChild(svg("rect", { x: 14, y: -12, width: 4, height: 24, rx: 1.5, class: "pidp-wheel" }));
    robotG.appendChild(svg("path", { d: "M0 -26 L6 -16 L-6 -16 Z", class: "pidp-arrow" }));
    field.appendChild(trail);
    field.appendChild(targetRay);
    field.appendChild(robotG);

    // Heading over time
    var chart = svg("svg", { viewBox: "0 0 400 200", class: "pidp-chart", role: "img", "aria-label": "Heading over time" });
    var band = svg("rect", { x: 34, width: 360, class: "pidp-band" });
    var targetLine = svg("line", { x1: 34, x2: 394, class: "pidp-target-line" });
    var curve = svg("path", { class: "pidp-curve", d: "" });
    var doneLine = svg("line", { y1: 8, y2: 176, class: "pidp-done-line" });
    var cursor = svg("circle", { r: 4, class: "pidp-cursor" });
    chart.appendChild(svg("line", { x1: 34, y1: 176, x2: 394, y2: 176, class: "pidp-axis" }));
    chart.appendChild(svg("line", { x1: 34, y1: 8, x2: 34, y2: 176, class: "pidp-axis" }));
    [0, 0.5, 1, 1.5, 2, 2.5].forEach(function (s) {
      var x = 34 + (s / 2.5) * 360;
      var label = svg("text", { x: x, y: 192, class: "pidp-tick", "text-anchor": "middle" });
      label.textContent = s + "s";
      chart.appendChild(label);
    });
    var yTop = svg("text", { x: 30, y: 14, class: "pidp-tick", "text-anchor": "end" });
    var yZero = svg("text", { x: 30, y: 179, class: "pidp-tick", "text-anchor": "end" });
    yZero.textContent = "0°";
    chart.appendChild(yTop);
    chart.appendChild(yZero);
    chart.appendChild(band);
    chart.appendChild(targetLine);
    chart.appendChild(doneLine);
    chart.appendChild(curve);
    chart.appendChild(cursor);

    // Controls
    function slider(key, label, min, max, step, hint) {
      var input = el("input", { type: "range", min: min, max: max, step: step, value: state[key], id: "pidp-" + key });
      var out = el("output", { for: "pidp-" + key });
      var box = el("div", { class: "pidp-slider" }, [
        el("label", { for: "pidp-" + key }, [el("span", { class: "pidp-name", text: label }), out]),
        input,
        el("small", { text: hint }),
      ]);
      function show() { out.textContent = key === "target" ? state[key] + "°" : String(state[key]); }
      input.addEventListener("input", function () { state[key] = parseFloat(input.value); show(); run(); });
      show();
      return { box: box, input: input, show: show };
    }
    var sliders = {
      kp: slider("kp", "kP", 0, 1.5, 0.01, "Pushes toward the target. More = faster, but overshoots"),
      ki: slider("ki", "kI", 0, 0.3, 0.005, "Adds up small leftover error. Fixes stopping just short"),
      kd: slider("kd", "kD", 0, 10, 0.1, "Brakes as it gets close. More = less overshoot"),
      target: slider("target", "Turn to", 15, 180, 5, "Target heading in degrees"),
    };

    var robotSelect = el("select", { "aria-label": "Robot" });
    Object.keys(ROBOTS).forEach(function (k) { robotSelect.appendChild(el("option", { value: k, text: ROBOTS[k].name })); });
    robotSelect.addEventListener("change", function () { state.robot = robotSelect.value; run(); });

    var presetRow = el("div", { class: "pidp-presets" });
    Object.keys(PRESETS).forEach(function (k) {
      var p = PRESETS[k];
      var b = el("button", { type: "button", class: "md-button", text: p.label });
      b.addEventListener("click", function () {
        state.kp = p.kp; state.ki = p.ki; state.kd = p.kd;
        ["kp", "ki", "kd"].forEach(function (s) { sliders[s].input.value = state[s]; sliders[s].show(); });
        run();
      });
      presetRow.appendChild(b);
    });
    var replay = el("button", { type: "button", class: "md-button md-button--primary", text: "▶ Replay" });
    replay.addEventListener("click", function () { animate(); });
    presetRow.appendChild(replay);

    var verdictTitle = el("strong", { class: "pidp-verdict-title" });
    var verdictText = el("span");
    var stats = el("div", { class: "pidp-stats" });
    var code = el("code");
    var copy = el("button", { type: "button", class: "pidp-copy", text: "Copy" });
    copy.addEventListener("click", function () {
      if (navigator.clipboard) navigator.clipboard.writeText(code.textContent).then(function () {
        copy.textContent = "Copied!";
        setTimeout(function () { copy.textContent = "Copy"; }, 1500);
      });
    });

    root.appendChild(el("div", { class: "pidp-stage" }, [
      el("div", { class: "pidp-field-wrap" }, [field]),
      el("div", { class: "pidp-chart-wrap" }, [chart,
        el("div", { class: "pidp-verdict" }, [verdictTitle, verdictText]), stats]),
    ]));
    root.appendChild(el("div", { class: "pidp-controls" }, [
      sliders.kp.box, sliders.ki.box, sliders.kd.box, sliders.target.box,
      el("div", { class: "pidp-slider" }, [el("label", {}, [el("span", { class: "pidp-name", text: "Robot" })]), robotSelect,
        el("small", { text: "Heavier robots need different numbers" })]),
    ]));
    root.appendChild(presetRow);
    root.appendChild(el("div", { class: "pidp-code" }, [el("span", { text: "In custom/src/robot-config.cpp:" }), el("pre", {}, [code]), copy]));

    var result, frame = null;
    var yMax = 1;
    function x(t) { return 34 + (t / result.timeLimit) * 360; }
    function y(h) { return 176 - (h / yMax) * 168; }

    function drawStatic() {
      yMax = Math.max(state.target * 1.25, Math.max.apply(null, result.headings) * 1.05);
      targetLine.setAttribute("y1", y(state.target));
      targetLine.setAttribute("y2", y(state.target));
      band.setAttribute("y", y(state.target + 1));
      band.setAttribute("height", y(state.target - 1) - y(state.target + 1));
      yTop.textContent = Math.round(yMax) + "°";
      if (result.arrivedAt !== null) {
        doneLine.setAttribute("x1", x(result.arrivedAt));
        doneLine.setAttribute("x2", x(result.arrivedAt));
        doneLine.style.display = "";
      } else {
        doneLine.style.display = "none";
      }
      targetRay.setAttribute("transform", "rotate(" + state.target + ")");

      var v = verdict(result);
      verdictTitle.textContent = v[0];
      verdictText.textContent = " " + v[1];
      root.dataset.verdict = v[0] === "Nice and crisp" ? "good" : "meh";
      stats.innerHTML = "";
      [["Done in", result.arrivedAt === null ? "timed out" : result.arrivedAt.toFixed(2) + " s"],
       ["Overshoot", result.overshoot.toFixed(1) + "°"],
       ["Final error", Math.abs(result.finalError).toFixed(1) + "°"]].forEach(function (s) {
        stats.appendChild(el("div", {}, [el("span", { text: s[0] }), el("b", { text: s[1] })]));
      });
      code.textContent = "double turn_kp = " + state.kp + ", turn_ki = " + state.ki + ", turn_kd = " + state.kd + ";";
    }

    function drawUpTo(n) {
      var d = "";
      var tr = "";
      for (var i = 0; i <= n; i++) {
        d += (i ? "L" : "M") + x(result.times[i]).toFixed(1) + " " + y(result.headings[i]).toFixed(1);
      }
      // Trail: the arc the front of the robot sweeps
      var steps = Math.min(n, 400);
      for (var k = 0; k <= steps; k++) {
        var h = result.headings[Math.round((k / Math.max(steps, 1)) * n)] * Math.PI / 180;
        tr += (k ? "L" : "M") + (Math.sin(h) * 40).toFixed(1) + " " + (-Math.cos(h) * 40).toFixed(1);
      }
      curve.setAttribute("d", d);
      trail.setAttribute("d", tr);
      cursor.setAttribute("cx", x(result.times[n]));
      cursor.setAttribute("cy", y(result.headings[n]));
      robotG.setAttribute("transform", "rotate(" + result.headings[n].toFixed(2) + ")");
    }

    function animate() {
      if (frame) cancelAnimationFrame(frame);
      var last = result.times.length - 1;
      if (reduceMotion) { drawUpTo(last); return; }
      var start = null;
      function tick(now) {
        if (start === null) start = now;
        var t = (now - start) / 1000;
        var n = Math.min(last, Math.round(t / STEP));
        drawUpTo(n);
        if (n < last) frame = requestAnimationFrame(tick);
      }
      frame = requestAnimationFrame(tick);
    }

    var timer = null;
    function run() {
      result = simulate(state);
      drawStatic();
      // Show the finished curve right away while dragging; replay the motion once you let go
      drawUpTo(result.times.length - 1);
      clearTimeout(timer);
      timer = setTimeout(animate, 350);
    }
    run();
  }

  if (typeof module !== "undefined" && module.exports) {
    module.exports = { simulate: simulate, verdict: verdict, ROBOTS: ROBOTS };
    return;
  }
  function mountAll() {
    document.querySelectorAll(".pid-playground:not([data-ready])").forEach(function (root) {
      root.setAttribute("data-ready", "1");
      mount(root);
    });
  }
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", mountAll);
  else mountAll();
})();
