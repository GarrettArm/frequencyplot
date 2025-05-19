package main

import (
	"fmt"
	"image/color"
	"log"
	"math"
	"math/cmplx"
	"sync"
	"time"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/dialog"
	"fyne.io/fyne/v2/widget"
	"github.com/gordonklaus/portaudio"
)

type FrequencyWindow struct {
	sync.RWMutex
	points          []float64
	frequencies     []float64
	cont            *fyne.Container
	window          fyne.Window
	inputChan       chan []float32
	bufferFull      bool
	stream          *portaudio.Stream
	generator       AudioGenerator
	genSelect       *widget.Select
	done            chan struct{}  // For animation
	audioDone       chan struct{}  // For audio processing
	wg              sync.WaitGroup
	mu              sync.RWMutex
}

func NewFrequencyWindow(window fyne.Window, generator AudioGenerator) fyne.CanvasObject {
	fw := &FrequencyWindow{
		window:     window,
		inputChan:  make(chan []float32, channelBufferSize),
		bufferFull: false,
		done:       make(chan struct{}),
		audioDone:  make(chan struct{}),
	}

	// Create a continuous frequency range from 20Hz to 20kHz
	startFreq := 20.0
	endFreq := 20000.0
	numPoints := 720

	for i := 0; i < numPoints; i++ {
		t := float64(i) / float64(numPoints-1)
		freq := startFreq * math.Pow(endFreq/startFreq, t)
		fw.frequencies = append(fw.frequencies, freq)
	}

	fw.points = make([]float64, len(fw.frequencies))

	// Create generator selection dropdown
	fw.genSelect = widget.NewSelect([]string{
		"Frequency Sweep",
		"Sine Wave (440 Hz)",
		"Microphone Input",
	}, fw.onGeneratorSelectionChanged)

	// Default to frequency sweep
	fw.genSelect.SetSelected("Sine Wave (440 Hz)")

	// Create main container with generator selection at top
	controls := container.NewHBox(
		widget.NewLabel("Generator:"),
		fw.genSelect,
	)

	plotContainer := container.NewWithoutLayout()
	fw.cont = plotContainer

	// Set dark background
	background := canvas.NewRectangle(color.NRGBA{R: 64, G: 64, B: 64, A: 255})
	background.Resize(window.Canvas().Size())
	window.Canvas().SetOnTypedKey(nil) // Remove key handler

	// Make background fill the container
	background.Move(fyne.NewPos(0, 0))
	plotContainer.Add(background)

	// Create an animation that will trigger updates
	anim := canvas.NewRectangle(color.Transparent)
	anim.Resize(fyne.NewSize(1, 1))
	anim.Move(fyne.NewPos(-1, -1))

	updateTicker := time.NewTicker(time.Second / 30)
	go func() {
		defer updateTicker.Stop()
		for {
			select {
			case <-updateTicker.C:
				func() {
					fw.Lock()
					defer fw.Unlock()
					fyne.Do(func() {
						fw.redrawPlot()
						anim.Refresh()
						fw.cont.Refresh()
					})
				}()
			case <-fw.done:
				return
			}
		}
	}()

	fw.cont.Add(anim)

	// Set default generator if none provided
	if generator == nil {
		generator = NewSineWaveGenerator(sampleRate, bufferSize, testFrequency)
	}
	fw.setGenerator(generator)

	// Update window close handler
	window.SetOnClosed(func() {
		fw.Close()
	})

	// Return combined container with controls at top
	return container.NewBorder(controls, nil, nil, nil, plotContainer)
}

func (f *FrequencyWindow) onGeneratorSelectionChanged(selected string) {
	// Stop existing audio processing
	if f.generator != nil {
		close(f.audioDone)
		f.wg.Wait()
		f.audioDone = make(chan struct{}) // Create new channel for new generator
	}

	var newGen AudioGenerator
	var err error

	switch selected {
	case "Frequency Sweep":
		newGen = NewFrequencySweepGenerator(
			sampleRate,
			bufferSize,
			20.0,    // Start at 20 Hz
			20000.0, // Sweep up to 20 kHz
			120.0,   // Complete sweep every 120 seconds (2 minutes)
		)
	case "Sine Wave (440 Hz)":
		newGen = NewSineWaveGenerator(sampleRate, bufferSize, testFrequency)
	case "Microphone Input":
		newGen, err = NewMicrophoneGenerator()
		if err != nil {
			dialog.ShowError(err, f.window)
			f.genSelect.SetSelected("Frequency Sweep")
			newGen = NewFrequencySweepGenerator(sampleRate, bufferSize, 20.0, 20000.0, 20.0)
		}
	}

	f.setGenerator(newGen)

	// Start audio processing with new generator
	if newGen != nil {
		go f.processAudio()
	}
}

func (f *FrequencyWindow) setGenerator(newGenerator AudioGenerator) {
	f.mu.Lock()
	defer f.mu.Unlock()

	if f.generator != nil {
		// Close old generator without holding the lock
		oldGen := f.generator
		f.generator = nil
		go func() {
			if err := oldGen.Close(); err != nil {
				log.Printf("Error closing generator: %v", err)
			}
		}()
	}

	f.generator = newGenerator
}

func (f *FrequencyWindow) Close() error {
	f.mu.Lock()
	if f.generator != nil {
		if err := f.generator.Close(); err != nil {
			f.mu.Unlock()
			return fmt.Errorf("error closing generator: %v", err)
		}
		f.generator = nil
	}
	f.mu.Unlock()

	close(f.done)
	close(f.audioDone)
	f.wg.Wait()
	return nil
}

func (f *FrequencyWindow) processData(samples []float64) {
    // Check input samples for validity
    maxSample := 0.0
    for _, s := range samples {
        if math.Abs(s) > maxSample {
            maxSample = math.Abs(s)
        }
    }
    if maxSample == 0 {
        fmt.Printf("Warning: All input samples are zero\n")
        return
    }
    if math.IsNaN(maxSample) {
        fmt.Printf("Warning: NaN detected in input samples\n")
        return
    }

    // Apply Hann window to reduce spectral leakage
    windowedSamples := make([]float64, len(samples))
    for i := range samples {
        // Hann window function: 0.5 * (1 - cos(2π * n/(N-1)))
        window := 0.5 * (1 - math.Cos(2*math.Pi*float64(i)/float64(len(samples)-1)))
        windowedSamples[i] = samples[i] * window
    }

    // Apply FFT to get frequency domain data
    spectrum := fft(windowedSamples)
    
    f.Lock()
    defer f.Unlock()
    
    // Calculate frequency resolution
    freqResolution := float64(sampleRate) / float64(len(spectrum))
    
    // Process each frequency bin with improved bin mapping
    for i := 0; i < len(f.frequencies); i++ {
        targetFreq := f.frequencies[i]
        
        // Find the closest FFT bin, only look at first half of spectrum (Nyquist theorem)
        centerBinIndex := int(math.Round(targetFreq / freqResolution))
        if centerBinIndex >= len(spectrum)/2 {
            // Frequency is above Nyquist frequency, should have zero magnitude
            f.points[i] = 0
            continue
        }
        
        // Use quadratic interpolation between three adjacent bins for better frequency accuracy
        magnitude := 0.0
        if centerBinIndex > 0 && centerBinIndex < len(spectrum)/2-1 {
            prev := cmplx.Abs(spectrum[centerBinIndex-1])
            center := cmplx.Abs(spectrum[centerBinIndex])
            next := cmplx.Abs(spectrum[centerBinIndex+1])
            
            // If center bin is a local maximum, use quadratic interpolation
            if center > prev && center > next {
                alpha := prev
                beta := center
                gamma := next
                p := 0.5 * (alpha - gamma) / (alpha - 2*beta + gamma)
                magnitude = beta - 0.25*(alpha-gamma)*p
            } else {
                magnitude = center
            }
        } else {
            magnitude = cmplx.Abs(spectrum[centerBinIndex])
        }
        
        // Normalize magnitude by window factor and FFT size
        magnitude = magnitude / (float64(len(spectrum)) * 0.375) // 0.375 is Hann window compensation
        
        // Convert to dB scale with improved range
        if magnitude > 0 {
            magnitude = 20 * math.Log10(magnitude)
            
            // Normalize to reasonable dB range (-80 to 0 dB)
            magnitude = (magnitude + 80) / 80 // Now in range 0 to 1
            if magnitude < 0 {
                magnitude = 0
            } else if magnitude > 1 {
                magnitude = 1
            }
        }
        
        f.points[i] = magnitude
    }

    // Print non-zero magnitude indices and their frequencies with threshold
    nonZeroIndices := make([]struct {
        idx int
        freq float64
        mag float64
    }, 0)
    threshold := 0.1 // Only show frequencies with significant magnitude
    for i, mag := range f.points {
        if mag > threshold {
            nonZeroIndices = append(nonZeroIndices, struct {
                idx int
                freq float64
                mag float64
            }{i, f.frequencies[i], mag})
        }
    }
    if len(nonZeroIndices) > 0 {
        fmt.Printf("Active bins (>.1): ")
        for _, v := range nonZeroIndices {
            fmt.Printf("%d(%.1fHz, %.2f) ", v.idx, v.freq, v.mag)
        }
        fmt.Println()
    }
}

func (f *FrequencyWindow) processAudio() {
	f.wg.Add(1)
	defer f.wg.Done()

	for {
		select {
		case <-f.audioDone:
			return
		default:
			f.mu.RLock()
			generator := f.generator
			f.mu.RUnlock()

			if generator == nil {
				time.Sleep(100 * time.Millisecond)
				continue
			}

			samples := generator.Generate()

			// Convert samples to float64
			samplesFloat64 := make([]float64, len(samples))
			for i, s := range samples {
				samplesFloat64[i] = float64(s)
			}

			// Process the samples
			f.processData(samplesFloat64)
		}
	}
}

func fft(input []float64) []complex128 {
	n := len(input)
	if n == 1 {
		return []complex128{complex(input[0], 0)}
	}

	// Split into even and odd
	even := make([]float64, n/2)
	odd := make([]float64, n/2)
	for i := 0; i < n/2; i++ {
		even[i] = input[2*i]
		odd[i] = input[2*i+1]
	}

	// Recursive FFT
	evenFFT := fft(even)
	oddFFT := fft(odd)

	// Combine results
	factor := -2 * math.Pi / float64(n)
	result := make([]complex128, n)
	for k := 0; k < n/2; k++ {
		term := cmplx.Rect(1, factor*float64(k))
		result[k] = evenFFT[k] + term*oddFFT[k]
		result[k+n/2] = evenFFT[k] - term*oddFFT[k]
	}

	return result
}

func (f *FrequencyWindow) getColorForOctave(octaveIndex int) color.NRGBA {
	colors := []color.NRGBA{
		{R: 0, G: 0, B: 255, A: 200},      // Deep Blue
		{R: 0, G: 128, B: 255, A: 200},    // Light Blue
		{R: 0, G: 255, B: 255, A: 200},    // Cyan
		{R: 0, G: 255, B: 128, A: 200},    // Blue-Green
		{R: 0, G: 255, B: 0, A: 200},      // Green
		{R: 128, G: 255, B: 0, A: 200},    // Yellow-Green
		{R: 255, G: 255, B: 0, A: 200},    // Yellow
		{R: 255, G: 192, B: 0, A: 200},    // Gold
		{R: 255, G: 128, B: 0, A: 200},    // Orange
		{R: 255, G: 64, B: 0, A: 200},     // Red-Orange
		{R: 255, G: 0, B: 0, A: 200},      // Red
		{R: 255, G: 0, B: 128, A: 200},    // Pink-Red
	}

	index := octaveIndex
	if index >= len(colors) {
		index = len(colors) - 1
	}
	return colors[index]
}

func (f *FrequencyWindow) redrawPlot() {
	// Remove old objects but keep the animation rectangle
	if len(f.cont.Objects) > 1 {
		f.cont.Objects = f.cont.Objects[:1]
	}

	width := float32(f.window.Canvas().Size().Width)
	height := float32(f.window.Canvas().Size().Height)
	size := float32(math.Min(float64(width), float64(height)))

	centerX := width / 2
	centerY := height / 2
	baseRadius := size * 0.4

	// Draw Archimedes spiral grid
	numTurns := float64(len(f.frequencies)) / 60.0 // One turn per 60 frequencies
	spiralSpacing := float64(baseRadius) / numTurns

	// Draw frequency data as a continuous line along the spiral
	for i := 1; i < len(f.frequencies); i++ {
		// Calculate positions for current and previous points
		prevProgress := float64(i-1) / float64(len(f.frequencies))
		progress := float64(i) / float64(len(f.frequencies))

		prevAngle := prevProgress * 2 * math.Pi * numTurns
		angle := progress * 2 * math.Pi * numTurns

		prevRadius := float32(spiralSpacing * prevAngle / (2 * math.Pi))
		radius := float32(spiralSpacing * angle / (2 * math.Pi))

		// Get magnitude for current point
		magnitude := f.points[i]

		// Calculate color based on frequency and use alpha/intensity for magnitude
		t := float64(i) / float64(len(f.frequencies))
		octaveIndex := int(t * 11) // Map to 12 color gradients
		baseColor := f.getColorForOctave(octaveIndex)

		 // Convert magnitude to alpha value - zero magnitude = invisible
		var alpha uint8
		if magnitude <= 0 {
			alpha = 0 // Make zero magnitude completely transparent
		} else {
			// Use exponential scaling for more dramatic effect on higher values
			// Scale magnitude to 0-1 range first (assuming typical log10 magnitude range)
			normalizedMag := (magnitude + 2) / 8 // Typical log10 magnitudes are -2 to 6
			if normalizedMag < 0 {
				normalizedMag = 0
			} else if normalizedMag > 1 {
				normalizedMag = 1
			}
			
			// Apply exponential scaling for more dramatic effect
			scaledMag := math.Pow(normalizedMag, 1.5)
			alpha = uint8(scaledMag * 255)
		}

		color := color.NRGBA{
			R: baseColor.R,
			G: baseColor.G,
			B: baseColor.B,
			A: alpha,
		}

		// Draw line segment with fixed spiral position
		line := canvas.NewLine(color)
		line.StrokeWidth = 2

		line.Position1 = fyne.NewPos(
			centerX+prevRadius*float32(math.Cos(prevAngle)),
			centerY+prevRadius*float32(math.Sin(prevAngle)))

		line.Position2 = fyne.NewPos(
			centerX+radius*float32(math.Cos(angle)),
			centerY+radius*float32(math.Sin(angle)))

		f.cont.Add(line)
	}

	// Add frequency labels
	numLabels := 8
	for i := 0; i < numLabels; i++ {
		progress := float64(i) / float64(numLabels)
		freqIndex := int(progress * float64(len(f.frequencies)))
		if freqIndex >= len(f.frequencies) {
			freqIndex = len(f.frequencies) - 1
		}

		angle := progress * 2 * math.Pi * numTurns
		radius := float32(spiralSpacing * angle / (2 * math.Pi))

		labelX := centerX + radius*float32(math.Cos(angle))
		labelY := centerY + radius*float32(math.Sin(angle))

		label := canvas.NewText(fmt.Sprintf("%.0fHz", f.frequencies[freqIndex]), color.White)
		label.TextSize = 8
		label.Alignment = fyne.TextAlignCenter
		label.Move(fyne.NewPos(labelX-20, labelY-10))
		f.cont.Add(label)
	}
}