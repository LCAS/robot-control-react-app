import { configureStore } from '@reduxjs/toolkit';
import connectionReducer from './slices/connectionSlice';
import robotReducer from './slices/robotSlice';
import logsReducer from './slices/logsSlice';
import navigationReducer from './slices/navigationSlice';

export const store = configureStore({
  reducer: {
    connection: connectionReducer,
    robot: robotReducer,
    logs: logsReducer,
    navigation: navigationReducer,
  },
  middleware: (getDefaultMiddleware) =>
    getDefaultMiddleware({
      serializableCheck: {
        ignoredActions: ['connection/setRosService'],
        ignoredPaths: ['connection.rosService'],
      },
    }),
});

// Remove TypeScript type exports - not needed in JavaScript
// export type RootState = ReturnType<typeof store.getState>;
// export type AppDispatch = typeof store.dispatch;