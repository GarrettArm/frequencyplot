package main

import (
	"math"
)

// FrequencySweepGenerator generates a sine wave that sweeps through frequencies
type FrequencySweepGenerator struct {
    sampleRate float64
    bufferSize int
    startFreq  float64
    endFreq    float64
    duration   float64  // Duration of sweep in seconds
    time       float64
    phase      float64 // Track phase to avoid discontinuities
}

func NewFrequencySweepGenerator(sampleRate float64, bufferSize int, startFreq, endFreq, duration float64) *FrequencySweepGenerator {
    return &FrequencySweepGenerator{
        sampleRate: sampleRate,
        bufferSize: bufferSize,
        startFreq:  startFreq,
        endFreq:    endFreq,
        duration:   duration,
        time:       0,
        phase:      0,
    }
}

func (g *FrequencySweepGenerator) Generate() []float32 {
    buffer := make([]float32, g.bufferSize)
    sampleTime := 1.0 / g.sampleRate

    for i := range buffer {
        // Calculate the current frequency using logarithmic sweep
        progress := g.time / g.duration
        if progress >= 1.0 {
            g.time = 0
            progress = 0
            // Keep phase continuous at wrap-around
            g.phase = math.Mod(g.phase, 2*math.Pi)
        }
        
        // Use exponential sweep for better frequency coverage
        currentFreq := g.startFreq * math.Pow(g.endFreq/g.startFreq, progress)
        
        // Use accumulated phase for better frequency stability
        g.phase += 2 * math.Pi * currentFreq * sampleTime
        sample := math.Sin(g.phase)
        
        // Apply amplitude envelope to avoid clicks at sweep reset
        envelope := 1.0
        fadeTime := 0.05 // 50ms fade
        if progress < fadeTime {
            envelope = progress / fadeTime
        } else if progress > (1.0 - fadeTime) {
            envelope = (1.0 - progress) / fadeTime
        }
        
        buffer[i] = float32(sample * envelope * 0.5) // 0.5 to prevent clipping
        g.time += sampleTime
    }

    return buffer
}

func (g *FrequencySweepGenerator) Close() error {
    // Nothing to clean up for frequency sweep generator
    return nil
}