// Home page header: a robot drives the RW logo's path and draws it as it goes, with a live
// position readout like the simulator's. Plays once; reduced-motion users see the finished logo.
(function () {
  "use strict";

  var DURATION = 3200; // ms
  // The logo's 512-unit badge stands for a 144" field centered on (0, 0)
  var INCHES_PER_UNIT = 144 / 512;

  function ease(p) {
    // Speed up, cruise, slow down: like a motion profile
    return p < 0.5 ? 2 * p * p : 1 - Math.pow(-2 * p + 2, 2) / 2;
  }

  function setup(art) {
    var ink = art.querySelector(".rwx-ink");
    var bot = art.querySelector(".rwx-bot");
    var button = art.querySelector(".rwx-replay-btn");
    var out = {};
    art.querySelectorAll(".rwx-readout b").forEach(function (b) { out[b.dataset.k] = b; });
    var length = ink.getTotalLength();
    var frame = null;

    function place(p) {
      ink.style.strokeDashoffset = String(1 - p);
      var at = Math.max(0.001, Math.min(length - 0.001, p * length));
      var a = ink.getPointAtLength(at - 0.5);
      var b = ink.getPointAtLength(at + 0.5);
      var heading = Math.atan2(b.x - a.x, -(b.y - a.y)) * 180 / Math.PI; // 0 = up, clockwise
      var pt = ink.getPointAtLength(at);
      bot.setAttribute("transform", "translate(" + pt.x.toFixed(1) + " " + pt.y.toFixed(1) + ") rotate(" + heading.toFixed(1) + ")");
      // Same transforms as the SVG groups around the path, then to field inches
      var sx = (pt.x - 14 - 256) * 0.92 + 256, sy = (pt.y + 10 - 256) * 0.92 + 256;
      out.x.textContent = ((sx - 256) * INCHES_PER_UNIT).toFixed(1);
      out.y.textContent = ((256 - sy) * INCHES_PER_UNIT).toFixed(1);
      out.h.textContent = Math.round((heading + 360) % 360) + "°";
    }

    function play() {
      if (frame) cancelAnimationFrame(frame);
      art.classList.remove("rwx-done");
      button.hidden = true;
      var start = null;
      function tick(now) {
        if (start === null) start = now;
        var p = Math.min(1, (now - start) / DURATION);
        place(ease(p));
        if (p < 1) {
          frame = requestAnimationFrame(tick);
        } else {
          art.classList.add("rwx-done");
          button.hidden = false;
        }
      }
      frame = requestAnimationFrame(tick);
    }

    button.addEventListener("click", play);
    if (window.matchMedia && matchMedia("(prefers-reduced-motion: reduce)").matches) {
      place(1);
      art.classList.add("rwx-done");
    } else {
      place(0);
      // Wait until it's on screen, so it doesn't play unseen
      if ("IntersectionObserver" in window) {
        var seen = new IntersectionObserver(function (entries) {
          if (entries[0].isIntersecting) { seen.disconnect(); play(); }
        }, { threshold: 0.4 });
        seen.observe(art);
      } else {
        play();
      }
    }
  }

  // Total downloads of every release asset. The page shows "1,000+" until this answers, and
  // keeps it if GitHub is unreachable or rate limited (60 requests an hour per visitor).
  function showDownloads() {
    var out = document.querySelector("[data-downloads]");
    if (!out || !window.fetch) return;
    var cached = null;
    try { cached = sessionStorage.getItem("rw-downloads"); } catch (e) { /* storage blocked */ }
    if (cached) { out.textContent = cached; return; }
    fetch("https://api.github.com/repos/richardbwang/RW-Template/releases?per_page=100")
      .then(function (res) { return res.ok ? res.json() : []; })
      .then(function (releases) {
        var total = 0;
        releases.forEach(function (r) { (r.assets || []).forEach(function (a) { total += a.download_count || 0; }); });
        if (total < 1000) return;
        var text = total.toLocaleString("en-US");
        out.textContent = text;
        try { sessionStorage.setItem("rw-downloads", text); } catch (e) { /* storage blocked */ }
      })
      .catch(function () { /* keep the fallback */ });
  }

  function init() {
    document.querySelectorAll(".rwx-hero__art").forEach(setup);
    showDownloads();
  }
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", init);
  else init();
})();
