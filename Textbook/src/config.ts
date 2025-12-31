// Central configuration for API and Auth URLs
// This handles the logic for switching between Localhost and Production automatically

const isDev = process.env.NODE_ENV === 'development';

export const config = {
    // Auth Server URL
    authUrl: isDev 
        ? 'http://localhost:4000/api/auth'
        : '/api/auth',

    // Python Backend URL (Bot & Profile Data)
    // Note: Once backend is deployed, replace the second URL with your Vercel/Render URL
    backendUrl: isDev 
        ? 'http://localhost:8001' 
        : 'https://mutahirhussain-ragbackend.hf.space',
};

export default config;