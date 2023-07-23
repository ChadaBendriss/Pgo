//Nom et prenom: CHADA BENDRISS
//Numero d'etudiant: 300266679
//Pour executer le code il faut ecrire dans le main:
//1st: go run main.go PointCloud1 0.99 0.05 0.1
//2nd: go run main.go PointCloud1_p0 0.99 0.05 0.1
//3rd: go run main.go PointCloud1_p0_p0 0.99 0.05 0.1

package main

import (
	"bufio"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"
)

// Structor of the Point3D
type Point3D struct {
	X float64
	Y float64
	Z float64
}

// Structor of the Plane3D
type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
}

// Structor of the Plane3DwSupport
type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

// reads an XYZ file and returns a slice of Point3D
func ReadXYZ(filename string) []Point3D {
	file, err := os.Open(filename)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	var points []Point3D
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		fields := strings.Split(scanner.Text(), "\t")
		if len(fields) != 3 {
			continue
		}

		x, err := strconv.ParseFloat(fields[0], 64)
		if err != nil {
			continue
		}

		y, err := strconv.ParseFloat(fields[1], 64)
		if err != nil {
			continue
		}

		z, err := strconv.ParseFloat(fields[2], 64)
		if err != nil {
			continue
		}

		points = append(points, Point3D{x, y, z})
	}

	return points
}

// saves a slice of Point3D into an XYZ file
func SaveXYZ(filename string, points []Point3D) {
	file, err := os.Create(filename)
	if err != nil {
		panic(err)
	}
	for i := 0; i < len(points); i++ {
		_, err := fmt.Fprintf(file, "%f\t%f\t%f\n", points[i].X, points[i].Y, points[i].Z)
		if err != nil {
			panic(err)
		}
	}
	file.Close()

}

// computes the distance between points p1 and p2
func (p1 *Point3D) GetDistance(p2 *Point3D) float64 {
	x := math.Sqrt((p2.X-p1.X)*(p2.X-p1.X) + (p2.Y-p1.Y)*(p2.Y-p1.Y) + (p2.Z-p1.Z)*(p2.Z-p1.Z))
	return x

}

// computes the plane defined by a slice of 3 points
func GetPlane(points []Point3D) Plane3D {

	a := (points[1].Y-points[0].Y)*(points[2].Z-points[0].Z) - (points[2].Y-points[0].Y)*(points[1].Z-points[0].Z)
	b := (points[1].Z-points[0].Z)*(points[2].X-points[0].X) - (points[2].Z-points[0].Z)*(points[1].X-points[0].X)
	c := (points[1].X-points[0].X)*(points[2].Y-points[0].Y) - (points[2].X-points[0].X)*(points[1].Y-points[0].Y)
	d := -(a*points[0].X + b*points[0].Y + c*points[0].Z)

	return Plane3D{a, b, c, d}

}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64, percentageOfPointsOnPlane float64) int {
	k := (int)(math.Log(1-confidence) / math.Log(1-math.Pow(percentageOfPointsOnPlane, 3)))
	return k
}

// computes the support of a plane in a slice of points
func GetSupport(plane Plane3D, points []Point3D, eps float64) Plane3DwSupport {
	support := 0

	for i := 0; i < len(points); i++ {
		distance := math.Abs(plane.A*points[i].X+plane.B*points[i].Y+plane.C*points[i].Z+plane.D) / math.Sqrt(plane.A*plane.A+plane.B*plane.B+plane.C*plane.C)

		if distance < eps {
			support = support + 1

		}

	}

	return Plane3DwSupport{plane, support}

}

// extracts the points that supports the given plane
// and returns them in a slice of points
func GetSupportingPoints(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var supportingPoints []Point3D
	for i := 0; i < len(points); i++ {
		distance := math.Abs(plane.A*points[i].X+plane.B*points[i].Y+plane.C*points[i].Z+plane.D) / math.Sqrt(plane.A*plane.A+plane.B*plane.B+plane.C*plane.C)
		if distance < eps {
			supportingPoints = append(supportingPoints, points[i])
		}
	}
	return supportingPoints
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var supportingPoints []Point3D
	for i := 0; i < len(points); i++ {
		distance := math.Abs(plane.A*points[i].X+plane.B*points[i].Y+plane.C*points[i].Z+plane.D) / math.Sqrt(plane.A*plane.A+plane.B*plane.B+plane.C*plane.C)
		if distance >= eps {
			supportingPoints = append(supportingPoints, points[i])
		}
	}
	return supportingPoints

}

//It randomly selects a point from the provided slice of Point3D (the input point cloud). Its output
//channel transmits instances of Point3D.

func RandomPointGenerator(points []Point3D) <-chan Point3D {
	outputPointStream := make(chan Point3D)
	go func() {
		rand.Seed(time.Now().UnixNano())
		for {
			index := rand.Intn(len(points))
			outputPointStream <- points[index]
		}
		close(outputPointStream)
	}()
	return outputPointStream

}

// It reads Point3D instances from its input channel and accumulate 3 points. Its output channel
// transmits arrays of Point3D (composed of three points).
func tripletGenerator(points <-chan Point3D) <-chan [3]Point3D {
	outputPointStream := make(chan [3]Point3D)
	go func() {
		triplet := [3]Point3D{}
		for p := range points {
			triplet[0], triplet[1], triplet[2] = triplet[1], triplet[2], p
			if triplet[0] != (Point3D{}) && triplet[1] != (Point3D{}) && triplet[2] != (Point3D{}) {
				outputPointStream <- triplet
			}

		}
		close(outputPointStream)
	}()
	return outputPointStream
}

// It reads arrays of Point3D and resend them. It automatically stops the pipeline after having received N
// arrays.
func takeN(points <-chan [3]Point3D, n int) <-chan [3]Point3D {
	outputPointStream := make(chan [3]Point3D)
	go func() {
		for i := 0; i < n; i++ {
			arrs := <-points
			outputPointStream <- arrs

		}
		close(outputPointStream)
	}()
	return outputPointStream

}

// It reads arrays of three Point3D and compute the plane defined by these points. Its output channel
// transmits Plane3D instances describing the computed plane parameters.
func planeEstimator(points <-chan [3]Point3D) <-chan Plane3D {
	outputPointStream := make(chan Plane3D)
	go func() {
		for i := range points {
			for k := 0; k < len(i); k++ {
				//fmt.Println(i[k])
			}
			plane := GetPlane(i[:])
			outputPointStream <- plane
			//fmt.Println(plane)
		}
		close(outputPointStream)
	}()
	return outputPointStream

}

//It counts the number of points in the provided slice of Point3D (the input point cloud) that supports
//the receive 3D plane. Its output channel transmits the plane parameters and the number of supporting
//points in a Point3DwSupport instance.

func supportingPointFinder(plane <-chan Plane3D, points []Point3D, eps float64) <-chan Plane3DwSupport {
	outputPointStream := make(chan Plane3DwSupport)
	go func() {
		for p := range plane {
			support := GetSupport(p, points, eps)
			outputPointStream <- support

		}
		close(outputPointStream)
	}()
	return outputPointStream
}

// It multiplexes the results receives from multiple channels into one output channel.
func fanIn(inputs ...<-chan Plane3DwSupport) <-chan Plane3DwSupport {
	output := make(chan Plane3DwSupport)
	var wg sync.WaitGroup
	wg.Add(len(inputs))
	for _, input := range inputs {
		go func(ch <-chan Plane3DwSupport) {
			for p := range ch {
				output <- p
			}
			wg.Done()
		}(input)
	}
	go func() {
		wg.Wait()
		close(output)
	}()
	return output
}

//It receives Plane3DwSupport instances and keep in memory the plane with the best support
//received so far. This component does not output values, it simply maintains the provided

func dominantPlaneIdentifier(input <-chan Plane3DwSupport, bestPlane *Plane3DwSupport) {
	for plane := range input {
		if plane.SupportSize > bestPlane.SupportSize {
			*bestPlane = plane
		}
	}
}

// Calls all the methods
func RansacPipeline(points []Point3D, n int, eps float64) Plane3DwSupport {
	bestSupport := Plane3DwSupport{}
	maxInliers := 0
	for i := 0; i < 1; i++ {
		fmt.Println("RandomPointGenerator")
		randPoints := RandomPointGenerator(points)
		fmt.Println("tripletGenerator")
		triplets := tripletGenerator(randPoints)
		fmt.Println("takeN")
		tripletsSubset := takeN(triplets, n)
		fmt.Println("planeEstimator")
		planes := planeEstimator(tripletsSubset)
		fmt.Println("supportingPointFinder")
		planesWithSupport := make([]<-chan Plane3DwSupport, 36000)
		for j := 0; j < 36000; j++ {
			planesWithSupport[j] = make(chan Plane3DwSupport)
			planesWithSupport[j] = supportingPointFinder(planes, points, eps)

		}
		fmt.Println("fanin")
		fanvar := fanIn(planesWithSupport...)

		dominantPlane := Plane3DwSupport{}
		fmt.Println("dominantPlaneIdentifier")
		dominantPlaneIdentifier(fanvar, &dominantPlane)
		fmt.Println("condition")
		if dominantPlane.SupportSize > maxInliers {
			maxInliers = dominantPlane.SupportSize
			bestSupport = dominantPlane

		}

	}
	return bestSupport

}

// go run main.go PointCloud1_p0_p0 0.99 0.05 0.1
// go run main.go PointCloud1_p0 0.99 0.05 0.1
// go run main.go PointCloud1 0.99 0.05 0.1
func main() {

	filename := os.Args[1]
	points := ReadXYZ(filename + ".xyz")

	//Step 2
	bestSupport := Plane3DwSupport{}

	//Step 3
	confidence, err := strconv.ParseFloat(os.Args[2], 64)
	if err != nil {
		log.Fatalf("Invalid confidence value")
	}
	percentage, err := strconv.ParseFloat(os.Args[3], 64)
	if err != nil {
		log.Fatalf("Invalid confidence value")
	}
	eps, err := strconv.ParseFloat(os.Args[4], 64)
	if err != nil {
		log.Fatalf("Invalid confidence value")
	}
	iterations := GetNumberOfIterations(confidence, percentage)

	//Step 4
	//stop := make(chan bool)
	start := time.Now()
	bestSupport = RansacPipeline(points, iterations, eps)
	elapsed := time.Since(start)
	fmt.Printf("Temps d'exécution : %s\n", elapsed)

	//Step 5
	// Save the supporting points of the identified dominant plane
	supportingPoints := GetSupportingPoints(bestSupport.Plane3D, points, eps)
	SaveXYZ(filename+"_p.xyz", supportingPoints)

	//Step 6
	// Save the original point cloud without the supporting points of the dominant plane
	removedPoints := RemovePlane(bestSupport.Plane3D, points, eps)
	SaveXYZ(filename+"_p0.xyz", removedPoints)

}
