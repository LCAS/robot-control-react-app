import React from 'react';
import { Provider } from 'react-redux';
import { store } from './store';
import './App.css';
import RobotController from './components/RobotController';

function App() {
  return (
    <Provider store={store}>
      <div className="container">
        <RobotController />
      </div>
    </Provider>
  );
}

export default App;