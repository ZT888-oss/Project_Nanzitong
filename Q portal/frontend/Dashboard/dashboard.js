const naviForm = document.getElementById("navi-form");
const studentMessage = document.getElementById("student-message");
const naviResponse = document.getElementById("navi-response");
const signOutButton = document.getElementById("sign-out-button");

signOutButton.addEventListener("click", function (event) {
    // Remove login information if you stored it
    localStorage.removeItem("loggedIn");

    // Go to the login page
    window.location.href = "login.html";
});

naviForm.addEventListener("submit", function (event) {
    event.preventDefault();

    const message = studentMessage.value.trim();

    if (!message) {
        return;
    }

    naviResponse.innerHTML = `
        <div class="support-icon">♡</div>

        <div>
            <h3>Thank you for sharing</h3>

            <p>
                It sounds like you may be carrying a lot right now.
                Try choosing one manageable next step and giving
                yourself permission to take a short break.
            </p>
        </div>
    `;
});