import React, { View, Text } from 'react-native';
import { ThemedText } from '@/components/themed-text';
import { SafeAreaView } from 'react-native-safe-area-context';

const Commands = () => {
  return (
    <SafeAreaView>
      <ThemedText>Commands</ThemedText>
    </SafeAreaView>
  );
};

export default Commands;