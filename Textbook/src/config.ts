// Central configuration for API and Auth URLs
// This handles the logic for switching between Localhost and Production automatically

const isDev = process.env.NODE_ENV === 'development';

export const config = {
    // Auth Server URL
    // During build (SSR), we need an absolute URL. In browser, we use relative to leverage proxy.
    authUrl: isDev 
        ? 'http://localhost:4000/api/auth'
        : (typeof window !== 'undefined' ? '/api/auth' : 'https://auth765.vercel.app/api/auth'),

    // Python Backend URL (Bot & Profile Data)
    // Note: Once backend is deployed, replace the second URL with your Vercel/Render URL
    backendUrl: isDev 
        ? 'http://localhost:8001' 
        : 'https://mutahirhussain-ragbackend.hf.space',
};

export default config;