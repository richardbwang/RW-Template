// Embeds simulator replays (sim/out/*.html, copied to docs/replays/ by build-replays.sh).
// Write <div class="rw-replay" data-src="../../replays/Name.html" data-title="Name"></div>
// in a page. The iframe grows to fit the replay and follows the site's light/dark switch.
(function () {
  function siteTheme() {
    return document.body.getAttribute("data-md-color-scheme") === "slate" ? "dark" : "light";
  }

  function applyTheme(frame) {
    var doc = frame.contentDocument;
    if (!doc || !doc.documentElement) return;
    doc.documentElement.setAttribute("data-theme", siteTheme());
    // The replay redraws its canvas on resize, which picks up the new colors
    frame.contentWindow.dispatchEvent(new Event("resize"));
  }

  function fitHeight(frame) {
    var doc = frame.contentDocument;
    if (doc && doc.body) frame.style.height = doc.documentElement.scrollHeight + "px";
  }

  function mount(box) {
    if (box.dataset.mounted) return;
    box.dataset.mounted = "1";
    var src = box.dataset.src;
    var title = box.dataset.title || "Simulator replay";

    fetch(src, { method: "HEAD" }).then(function (res) {
      if (!res.ok) throw new Error(res.status);
      var frame = document.createElement("iframe");
      frame.src = src;
      frame.title = title + " replay";
      frame.loading = "lazy";
      frame.addEventListener("load", function () {
        applyTheme(frame);
        fitHeight(frame);
        if (window.ResizeObserver && frame.contentDocument) {
          new ResizeObserver(function () { fitHeight(frame); }).observe(frame.contentDocument.body);
        }
      });
      box.appendChild(frame);

      var link = document.createElement("a");
      link.href = src;
      link.target = "_blank";
      link.rel = "noopener";
      link.className = "rw-replay__open";
      link.textContent = "Open full screen ↗";
      box.appendChild(link);
    }).catch(function () {
      box.classList.add("rw-replay--missing");
      box.textContent = "This replay isn't built yet. Run sh docs/build-replays.sh, then reload.";
    });
  }

  function mountAll() {
    document.querySelectorAll(".rw-replay").forEach(mount);
  }

  new MutationObserver(function () {
    document.querySelectorAll(".rw-replay iframe").forEach(applyTheme);
  }).observe(document.body, { attributes: true, attributeFilter: ["data-md-color-scheme"] });

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", mountAll);
  else mountAll();
})();
