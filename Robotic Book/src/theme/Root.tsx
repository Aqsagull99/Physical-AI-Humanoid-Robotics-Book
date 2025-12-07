import React, { useEffect } from 'react';
import Chatbot from '../components/Chatbot/Chatbot';

// Default implementation, that you can customize
function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}

export default Root;