import dotenv from 'dotenv';
dotenv.config({ path: '../.env' });

import express from 'express';
import cors from 'cors';
import { auth } from './auth.js';
import { toNodeHandler } from 'better-auth/node';

const app = express();
const PORT = 4000;

// Enable CORS - Allow all origins for now to debug, then restrict
app.use(cors({
    origin: true, // Allow any origin temporarily to fix connection
    credentials: true,
    allowedHeaders: ['Content-Type', 'Authorization'],
}));

app.use(express.json());

console.log("Initializing Better Auth...");
const authHandler = toNodeHandler(auth);

// Root route to check if server is running
app.get('/', (req, res) => {
    res.send('Auth Server is Running!');
});

app.use('/api/auth', (req, res, next) => {
    authHandler(req, res).catch(err => {
        console.error("Better Auth Error:", err);
        next(err);
    });
});

// Only listen if NOT running on Vercel (Vercel exports the app instead)
if (process.env.NODE_ENV !== 'production') {
    const server = app.listen(PORT, () => {
        console.log(`Auth server running on http://localhost:${PORT}`);
        console.log("Ready to handle authentication requests");
    });

    server.on('error', (err) => {
        console.error("Server failed to start:", err);
    });
}

export default app;
