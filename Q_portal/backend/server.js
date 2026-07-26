const express = require("express"); //webserver
const Database = require("better-sqlite3"); //database
const bcrypt = require("bcrypt"); //password_hash
const path = require("path"); //file path

const app = express(); //save Express application into app
const PORT = 3000;

const db = new Database("users.db");

//create user_info table only do when it not exist
db.prepare(`
    CREATE TABLE IF NOT EXISTS user_info (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        username TEXT NOT NULL,
        email TEXT NOT NULL UNIQUE,
        password_hash TEXT NOT NULL,
        preferred_language TEXT NOT NULL DEFAULT 'en',
        created_at DATETIME DEFAULT CURRENT_TIMESTAMP
    )
`).run();

//convert incoming JSON back to javascript object that express can understand
app.use(express.json());  //For incoming requests, use Express's JSON-reading functionality.

app.use(express.urlencoded({ extended: true }));

//For incoming requests, use Express's static-file-serving functionality
app.use(
    express.static(
        path.join(__dirname, "../frontend") //Serve all the files inside the frontend folder
    )
);


// examine register information when frontend send information to backend
app.post("/api/register", async (request, response) => {
    try {
        const {
            username,
            email,
            password,
            preferredLanguage = "en"
        } = request.body;

        if (!username || !email || !password) {
            return response.status(400).json({
                success: false,
                message: "All fields are required."
            });
        }

        const existingUser = db.prepare(`
            SELECT id
            FROM user_info
            WHERE username = ? OR email = ?
        `).get(username, email);

        if (existingUser) {
            return response.status(409).json({
                success: false,
                message: "Username or email already exists."
            });
        }

        const passwordHash = await bcrypt.hash(password, 10);

        const result = db.prepare(`
            INSERT INTO user_info (
                username,
                email,
                password_hash,
                preferred_language
            )
            VALUES (?, ?, ?, ?)
        `).run(
            username,
            email,
            passwordHash,
            preferredLanguage
        );

        response.status(201).json({
            success: true,
            userId: result.lastInsertRowid,
            message: "Account created successfully."
        });
    } catch (error) {
        console.error(error);

        response.status(500).json({
            success: false,
            message: "Server error."
        });
    }
});


//examine login information when frontend send request
app.post("/api/login", async (request, response) => {
    try {
        const { username, password } = request.body;

        if (!username || !password) {
            return response.status(400).json({
                success: false,
                message: "Username and password are required."
            });
        }

        const user = db.prepare(`
            SELECT
                id,
                username,
                email,
                password_hash,
                preferred_language
            FROM user_info
            WHERE username = ? OR email = ?
        `).get(username, username);

        if (!user) {
            return response.status(401).json({
                success: false,
                message: "Invalid username or password."
            });
        }

        const passwordMatches = await bcrypt.compare(
            password,
            user.password_hash
        );

        if (!passwordMatches) {
            return response.status(401).json({
                success: false,
                message: "Invalid username or password."
            });
        }

        response.json({
            success: true,
            user: {
                id: user.id,
                username: user.username,
                email: user.email,
                preferredLanguage: user.preferred_language
            }
        });
    } catch (error) {
        console.error(error);

        response.status(500).json({
            success: false,
            message: "Server error."
        });
    }
});





app.listen(PORT, () => {
    console.log(`Server running at http://localhost:${PORT}`);
});