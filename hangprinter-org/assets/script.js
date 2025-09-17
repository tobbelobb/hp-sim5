let burger = document.querySelector(".burger")
let mobileMenu = document.querySelector("#mobile-menu")
let blurOverlay = document.querySelector(".blur-overlay")

menuIsClosed = true

burger.addEventListener("click", async () => {

    if (menuIsClosed) {
        mobileMenu.style.display = "flex"
        blurOverlay.style.display = "block"
        burger.style.transform = "rotate(90deg)"

        const openMenu = mobileMenu.animate(
            { transform: "translateY(650px)" },
            { duration: 600, easing: "ease-in-out", fill: "forwards" }
        );
        await openMenu.finished;
        openMenu.commitStyles()
        openMenu.cancel;

        menuIsClosed = false
    }
    else {
        blurOverlay.style.display = "none"
        burger.style.transform = "rotate(0)"

        const closeMenu = mobileMenu.animate(
            { transform: "translateY(0px)" },
            { duration: 600, easing: "ease-in-out", fill: "forwards" }
        );
        await closeMenu.finished;
        closeMenu.commitStyles()
        closeMenu.cancel;

        menuIsClosed = true
    }


})


blurOverlay.addEventListener("click", async () => {
    if (menuIsClosed == false) {
        // mobileMenu.style.display = "none"
        blurOverlay.style.display = "none"
        burger.style.transform = "rotate(0)"

        const closeMenu = mobileMenu.animate(
            { transform: "translateY(0px)" },
            { duration: 600, easing: "ease-in-out", fill: "forwards" }
        );
        await closeMenu.finished;
        closeMenu.commitStyles()
        closeMenu.cancel;

        menuIsClosed = true
    }
})