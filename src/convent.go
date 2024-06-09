package src

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"strconv"
	"strings"
	"time"

	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
)

// WriteJointRotations writes joint rotation data to a CSV file.
func WriteJointRotations(bvhTree *BvhTree, filePath string, endSites bool) bool {
	startTime := time.Now()

	timeCol := make([]float64, bvhTree.NFrames())
	for i := 0; i < bvhTree.NFrames(); i++ {
		timeCol[i] = float64(i) * bvhTree.FrameTime()
	}

	data := make([][]float64, len(bvhTree.GetJoints(endSites)))
	header := []string{"time"}

	for i, joint := range bvhTree.GetJoints(endSites) {
		channels := bvhTree.JointChannels(joint.Name())
		rotationChannels := make([]string, 0)

		for _, channel := range channels {
			if strings.HasSuffix(channel, "rotation") {
				rotationChannels = append(rotationChannels, channel)
			}
		}
		JointChannels := bvhTree.FramesJointChannels(joint.Name(), rotationChannels, 0.0)[i]

		header = append(header, fmt.Sprintf("%s.%s", joint.Name(), strings.ToLower(string(rotationChannels[0][0]))))
		data[i] = append(data[i], timeCol...)
		data[i] = append(data[i], JointChannels...)
	}

	endTime := time.Now()
	fmt.Printf("rotations: %s\n", endTime.Sub(startTime))

	file, err := os.Create(filePath)
	if err != nil {
		fmt.Printf("ERROR(%s): Could not write to file %s.\nMake sure you have writing permissions.\n", err, filePath)
		return false
	}
	defer file.Close()

	writer := bufio.NewWriter(file)
	defer writer.Flush()

	fmt.Fprintf(writer, "%s\n", strings.Join(header, ","))
	for _, row := range data {
		for j, v := range row {
			if j > 0 {
				fmt.Fprintf(writer, ",")
			}
			fmt.Fprintf(writer, "%10.5f", v)
		}
		fmt.Fprintf(writer, "\n")
	}

	return true
}

// WriteJointPositions writes joint world position data to a CSV file.
func WriteJointPositions(bvhTree *BvhTree, filePath string, scale float64, endSites bool) bool {
	startTime := time.Now()

	timeCol := make([]float64, bvhTree.NFrames())
	for i := 0; i < bvhTree.NFrames(); i++ {
		timeCol[i] = float64(i) * bvhTree.FrameTime()
	}

	data := make([][]float64, len(bvhTree.GetJoints(true)))
	header := []string{"time"}

	var getWorldPositions func(joint *BvhNode)
	getWorldPositions = func(joint *BvhNode) {
		var offset []float64
		if joint.Value[0] == "End" {
			offset = []float64{0, 0, 0}
		} else {
			offset = bvhTree.JointOffset(joint.Name())
		}
		for _, axis := range []string{"x", "y", "z"} {
			header = append(header, fmt.Sprintf("%s.%s", joint.Name(), axis))
		}
		for i := 0; i < bvhTree.NFrames(); i++ {
			data[bvhTree.GetJointIndex(joint.Name())] = append(data[bvhTree.GetJointIndex(joint.Name())], timeCol[i])
			data[bvhTree.GetJointIndex(joint.Name())] = append(data[bvhTree.GetJointIndex(joint.Name())], scale*offset[0])
			data[bvhTree.GetJointIndex(joint.Name())] = append(data[bvhTree.GetJointIndex(joint.Name())], scale*offset[1])
			data[bvhTree.GetJointIndex(joint.Name())] = append(data[bvhTree.GetJointIndex(joint.Name())], scale*offset[2])
		}
		for _, child := range bvhTree.JointChildren(joint.Name()) {
			if endSites {
				getWorldPositions(child)
			}
			if child.Value[0] == "JOINT" {
				getWorldPositions(child)
			}
		}
	}
	getWorldPositions(bvhTree.Search("ROOT")[0])

	endTime := time.Now()
	fmt.Printf("positions: %s\n", endTime.Sub(startTime))

	file, err := os.Create(filePath)
	if err != nil {
		fmt.Printf("ERROR(%s): Could not write to file %s.\nMake sure you have writing permissions.\n", err, filePath)
		return false
	}
	defer file.Close()

	writer := bufio.NewWriter(file)
	defer writer.Flush()

	fmt.Fprintf(writer, "%s\n", strings.Join(header, ","))
	for _, row := range data {
		for j, v := range row {
			if j > 0 {
				fmt.Fprintf(writer, ",")
			}
			fmt.Fprintf(writer, "%10.5f", v)
		}
		fmt.Fprintf(writer, "\n")
	}

	return true
}

// WriteJointHierarchy writes joint hierarchy data to a CSV file.
func WriteJointHierarchy(bvhTree *BvhTree, filePath string, scale float64) bool {
	startTime := time.Now()

	var data [][]string
	for _, joint := range bvhTree.GetJoints(true) {
		jointName := joint.Name()
		parentName := ""
		if bvhTree.JointParent(jointName) != nil {
			parentName = bvhTree.JointParent(jointName).Name()
		}
		offset := bvhTree.JointOffset(jointName)
		row := []string{jointName, parentName, fmt.Sprintf("%f", scale*offset[0]), fmt.Sprintf("%f", scale*offset[1]), fmt.Sprintf("%f", scale*offset[2])}
		data = append(data, row)
	}

	endTime := time.Now()
	fmt.Printf("hierarchy: %s\n", endTime.Sub(startTime))

	file, err := os.Create(filePath)
	if err != nil {
		fmt.Printf("ERROR(%s): Could not write to file %s.\nMake sure you have writing permissions.\n", err, filePath)
		return false
	}
	defer file.Close()

	writer := bufio.NewWriter(file)
	defer writer.Flush()

	fmt.Fprintf(writer, "joint,parent,offset.x,offset.y,offset.z\n")
	for _, row := range data {
		fmt.Fprintf(writer, "%s,%s,%s,%s,%s\n", row[0], row[1], row[2], row[3], row[4])
	}

	return true
}

// Bvh2Csv converts a BVH file to CSV format.
func Bvh2Csv(bvhPath string, dstDirpath string, scale float64, exportRotation bool, exportPosition bool, exportHierarchy bool, endSites bool) bool {
	startTime := time.Now()

	var file *os.File
	var err error
	if file, err = os.Open(bvhPath); err != nil {
		fmt.Printf("ERROR %s: Could not open file %s\n", err, bvhPath)
		return false
	}
	defer file.Close()

	data, _ := io.ReadAll(file)
	bvhTree := NewBvhTree(string(data))

	endTime := time.Now()
	fmt.Printf("file read: %s\n", endTime.Sub(startTime))

	var posSuccess, rotSuccess, hierarchySuccess bool
	posSuccess = false
	rotSuccess = false
	hierarchySuccess = false

	if exportPosition {
		posSuccess = WriteJointPositions(bvhTree, fmt.Sprintf("%s/%s_pos.csv", dstDirpath, bvhPath), scale, endSites)
	}

	if exportRotation {
		rotSuccess = WriteJointRotations(bvhTree, fmt.Sprintf("%s/%s_rot.csv", dstDirpath, bvhPath), endSites)
	}

	if exportHierarchy {
		hierarchySuccess = WriteJointHierarchy(bvhTree, fmt.Sprintf("%s/%s_hierarchy.csv", dstDirpath, bvhPath), scale)
	}

	return bool(posSuccess && rotSuccess && hierarchySuccess)
}

// PlotCsv plots data from a CSV file.
func PlotCsv(filePath string, xColumn int, yColumn int) error {
	file, err := os.Open(filePath)
	if err != nil {
		return err
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	scanner.Scan() // Skip header line
	header := strings.Split(scanner.Text(), ",")

	var pts plotter.XYs
	for scanner.Scan() {
		row := strings.Split(scanner.Text(), ",")
		x, err := strconv.ParseFloat(row[xColumn], 64)
		if err != nil {
			return err
		}
		y, err := strconv.ParseFloat(row[yColumn], 64)
		if err != nil {
			return err
		}
		pts = append(pts, plotter.XY{X: x, Y: y})
	}

	p := plot.New()
	s, err := plotter.NewScatter(pts)
	if err != nil {
		return err
	}
	p.Add(s)

	p.X.Label.Text = header[xColumn]
	p.Y.Label.Text = header[yColumn]

	if err := p.Save(4*vg.Inch, 4*vg.Inch, "plot.png"); err != nil {
		return err
	}

	return nil
}
