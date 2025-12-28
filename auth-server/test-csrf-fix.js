// Simple test server to verify CSRF endpoint
import express from 'express';
import cors from 'cors';
import dotenv from 'dotenv';

dotenv.config({ path: '../.env' });

const app = express();
const PORT = 4000;

// Enable CORS
app.use(cors({
    origin: ['http://localhost:3000', 'http://localhost:8000'],
    credentials: true,
    allowedHeaders: ['Content-Type', 'Authorization'],
}));

// Parse JSON bodies
app.use(express.json());

// Test CSRF endpoint that returns the expected format
app.get('/api/auth/csrf', (req, res) => {
    console.log('CSRF endpoint hit!');
    res.json({
        csrf_token: 'test-csrf-token'
    });
});

// Test other auth endpoints
app.post('/api/auth/sign-up/email', (req, res) => {
    console.log('Sign-up endpoint hit!');
    res.json({
        token: 'test-token',
        user: {
            id: 'test-id',
            email: req.body.email,
            name: req.body.name
        }
    });
});

app.post('/api/auth/sign-in/email', (req, res) => {
    console.log('Sign-in endpoint hit!');
    res.json({
        token: 'test-token',
        user: {
            id: 'test-id',
            email: req.body.email
        }
    });
});

// Health check
app.get('/api/auth/health', (req, res) => {
    res.json({ status: 'ok' });
});

app.listen(PORT, () => {
    console.log(`Test server running on http://localhost:${PORT}`);
    console.log('CSRF endpoint available at: http://localhost:4000/api/auth/csrf');
});