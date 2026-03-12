import React, { View, Text } from 'react-native';
import { ThemedText } from '@/components/themed-text';
import { SafeAreaView } from 'react-native-safe-area-context';

export default function HomeScreen() {
  return (
    <SafeAreaView>
        <ThemedText>Visualization</ThemedText>
    </SafeAreaView>
  );
}