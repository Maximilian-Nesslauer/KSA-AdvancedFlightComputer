// Sizes each embedded visualiser (<!-- viz: NAME --> markers) to its content, so it never scrolls inside the page.
// The HEIGHT in the marker is only the size before the visualiser loads.
document$.subscribe(() => {
  document.querySelectorAll(".afc-viz iframe").forEach((frame) => {
    const fit = () => {
      try {
        const body = frame.contentDocument && frame.contentDocument.body;
        if (!body) return;
        const style = frame.contentWindow.getComputedStyle(body);
        const height =
          body.getBoundingClientRect().height +
          parseFloat(style.marginTop) +
          parseFloat(style.marginBottom);
        if (height > 0) frame.style.height = Math.ceil(height) + "px";
      } catch (e) {
        // Not same-origin: keep the marker's height.
      }
    };
    const attach = () => {
      fit();
      try {
        new frame.contentWindow.ResizeObserver(fit).observe(frame.contentDocument.body);
      } catch (e) {
        // Keep the first fit.
      }
    };
    frame.addEventListener("load", attach);
    try {
      const doc = frame.contentDocument;
      if (doc && doc.readyState === "complete" && doc.URL !== "about:blank") attach();
    } catch (e) {
      // Wait for the load event.
    }
  });
});
