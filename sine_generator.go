package main

import (
	"math"
)

// SineWaveGenerator generates sine waves with harmonics
type SineWaveGenerator struct {
	sampleRate float64
	bufferSize int
	frequency  float64
	time       float64
	harmonics  []struct {
		multiple  float64  // frequency multiple of fundamental
		amplitude float64 // relative amplitude (0-1)
	}
}

func NewSineWaveGenerator(sampleRate float64, bufferSize int, frequency float64) *SineWaveGenerator {
	return &SineWaveGenerator{
		sampleRate: sampleRate,
		bufferSize: bufferSize,
		frequency:  frequency,
		time:      0,
		harmonics: []struct {
			multiple  float64
			amplitude float64
		}{
			{1.0, 1.0},    // fundamental only at full amplitude, no harmonics
		},
	}
}

func (g *SineWaveGenerator) Generate() []float32 {
	buffer := make([]float32, g.bufferSize)
	sampleTime := 1.0 / g.sampleRate

	// Generate sine wave for each harmonic
	for i := range buffer {
		var sample float64
		for _, h := range g.harmonics {
			// Each harmonic is a sine wave at freq * multiple with given amplitude
			sample += h.amplitude * math.Sin(2 * math.Pi * g.frequency * h.multiple * g.time)
		}
		buffer[i] = float32(sample * 0.5) // 0.5 to prevent clipping
		g.time += sampleTime
	}

	return buffer
}

// SetHarmonics allows modifying the frequency components of the sine wave
func (g *SineWaveGenerator) SetHarmonics(harmonics []struct {
    multiple  float64
    amplitude float64
}) {
    g.harmonics = harmonics
}

func (g *SineWaveGenerator) Close() error {
	// Nothing to clean up for sine wave generator
	return nil
}