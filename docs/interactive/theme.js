// Shared by every visualiser under docs/interactive/.
// Embedded in a docs page, it follows the site's light/dark switch by setting data-theme on <html>.
// Opened on its own, it leaves data-theme unset, so the page follows prefers-color-scheme.
(function () {
  const root = document.documentElement;

  function siteScheme() {
    try {
      if (window.parent === window) return null;
      const scheme = window.parent.document.body.getAttribute("data-md-color-scheme");
      if (scheme === "slate") return "dark";
      if (scheme === "default") return "light";
    } catch (e) {
      // Cross-origin parent: fall back to the system setting.
    }
    return null;
  }

  function apply() {
    const theme = siteScheme();
    if (theme) root.setAttribute("data-theme", theme);
    else root.removeAttribute("data-theme");
    window.dispatchEvent(new Event("afc-theme"));
  }

  apply();
  try {
    if (window.parent !== window) {
      new MutationObserver(apply).observe(window.parent.document.body, {
        attributes: true,
        attributeFilter: ["data-md-color-scheme"],
      });
    }
  } catch (e) {
    // Not reachable; the system setting applies.
  }
})();
